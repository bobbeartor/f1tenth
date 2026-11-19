#include "multiclass_yolo/yolox_detector.hpp"
#include "multiclass_yolo/distance.hpp"
#include "multiclass_yolo/rgb_distance_model.hpp"
#include "multiclass_yolo/distance_live_preview.hpp"
#include <point_cloud/depth_source.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cstring>
#include <deque>
#include <filesystem>
#include <thread>

namespace mc=multiclass_yolo;
#ifndef MULTICLASS_YOLO_MODEL_PACKAGE
#define MULTICLASS_YOLO_MODEL_PACKAGE "multiclass_yolo"
#endif
using Clock=std::chrono::steady_clock;
static double seconds(Clock::time_point t) {return std::chrono::duration<double>(t.time_since_epoch()).count();}
static double elapsed(Clock::time_point t) {return (seconds(Clock::now())-seconds(t))*1000;}
static point_cloud::Intrinsics intrinsics(dai::ImgFrame& f) {
  if(!f.getTransformation().isValid()) throw std::runtime_error("Missing calibrated frame geometry");
  auto k=f.getTransformation().getIntrinsicMatrix();
  point_cloud::Intrinsics out{k[0][0],k[1][1],k[0][2],k[1][2]};
  if(!std::isfinite(out.fx+out.fy+out.cx+out.cy) || out.fx<=0 || out.fy<=0)
    throw std::runtime_error("Invalid intrinsics");
  return out;
}
static point_cloud::RigidTransform inverse(point_cloud::RigidTransform t) {
  auto r=t;
  for(int i=0;i<3;++i) for(int j=0;j<3;++j) r.rotation[i*3+j]=t.rotation[j*3+i];
  r.translation=point_cloud::rotate(r.rotation,t.translation);
  for(auto& v:r.translation) v=-v;
  return r;
}
// Calibration-aware forward warp, nearest-surface z buffer. No hole filling.
static cv::Mat alignDepth(dai::ImgFrame& rgb,dai::ImgFrame& depth,const dai::CalibrationHandler& calibration) {
  auto dk=intrinsics(depth), rk=intrinsics(rgb);
  auto t=point_cloud::compose(inverse(point_cloud::rgbFromFrame(rgb,calibration)),
    point_cloud::rgbFromFrame(depth,calibration));
  cv::Mat out(rgb.getHeight(),rgb.getWidth(),CV_64F,cv::Scalar(0));
  const auto& bytes=depth.getData(); auto stride=depth.getStride();
  if(depth.getType()!=dai::ImgFrame::Type::RAW16 || stride<depth.getWidth()*2 ||
     bytes.size()<std::size_t(stride)*depth.getHeight()) throw std::runtime_error("Invalid RAW16 depth");
  for(unsigned y=0;y<depth.getHeight();++y) for(unsigned x=0;x<depth.getWidth();++x) {
    std::uint16_t mm; std::memcpy(&mm,bytes.data()+y*stride+x*2,2); if(!mm) continue;
    double z=mm*.001;
    auto p=point_cloud::rotate(t.rotation,point_cloud::Vector3{(x-dk.cx)*z/dk.fx,(y-dk.cy)*z/dk.fy,z});
    for(int i=0;i<3;++i) p[i]+=t.translation[i];
    if(p[2]<=0 || !std::isfinite(p[0]+p[1]+p[2])) continue;
    double u=rk.fx*p[0]/p[2]+rk.cx, v=rk.fy*p[1]/p[2]+rk.cy;
    if(u<0 || v<0 || u>=out.cols-.5 || v>=out.rows-.5) continue;
    auto& target=out.at<double>(int(std::lround(v)),int(std::lround(u)));
    if(target==0 || p[2]<target) target=p[2];
  }
  return out;
}
class DistanceNode:public rclcpp::Node {
 public:
  DistanceNode():Node("yolo_distance") {}
  void run() {
    auto mode=declare_parameter<std::string>("mode","collect");
    if(mode!="collect" && mode!="infer" && mode!="evaluate" && mode!="rgb_infer" && mode!="rgb_compare")
      throw std::invalid_argument("mode: collect/infer/evaluate/rgb_infer/rgb_compare");
    const bool stereo_enabled=mode=="collect" || mode=="evaluate" || mode=="rgb_compare";
    int width=declare_parameter<int>("width",1280), height=declare_parameter<int>("height",800);
    double fps=declare_parameter<double>("fps",30), sync_ms=declare_parameter<double>("sync_ms",20);
    bool preview=declare_parameter<bool>("preview",true);
    auto sequence=declare_parameter<std::string>("sequence","");
    auto directory=declare_parameter<std::string>("output_directory","distance_data");
    if(width<16 || height<16 || width>4096 || height>4096 || !std::isfinite(fps) || fps<=0 || fps>120 ||
       !std::isfinite(sync_ms) || sync_ms<=0 || sync_ms>100) throw std::invalid_argument("Invalid stream settings");
    if(mode=="collect" && (sequence.empty() || sequence.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos))
      throw std::invalid_argument("collect requires a unique sequence (letters, digits, _ or -)");
    mc::DepthQuality quality;
    quality.core=declare_parameter<double>("core_fraction",.6);
    quality.min_ratio=declare_parameter<double>("min_valid_ratio",.1);
    quality.max_relative_spread=declare_parameter<double>("max_relative_spread",.15);
    quality.min_pixels=declare_parameter<int>("min_pixels",20);
    quality.min_m=declare_parameter<double>("min_depth_m",.2);
    quality.max_m=declare_parameter<double>("max_depth_m",10);
    if(!std::isfinite(quality.core+quality.min_ratio+quality.max_relative_spread+quality.min_m+quality.max_m) ||
       quality.core<=0 || quality.core>1 || quality.min_ratio<=0 || quality.min_ratio>1 || quality.max_relative_spread<=0 ||
       quality.min_pixels<1 || quality.min_m<=0 || quality.max_m<=quality.min_m) throw std::invalid_argument("Invalid depth quality settings");
    auto share=ament_index_cpp::get_package_share_directory(MULTICLASS_YOLO_MODEL_PACKAGE);
    auto model_path=declare_parameter<std::string>("yolo_model",share+"/models/multiclass_yolox_tiny_512x320_holdout10.onnx");
    auto backend=declare_parameter<std::string>("backend","TENSORRT");
    const double confidence=declare_parameter<double>("confidence_threshold",.4);
    const double nms=declare_parameter<double>("nms_threshold",.65);
    if(!std::isfinite(confidence+nms) || confidence<0 || confidence>1 || nms<0 || nms>1)
      throw std::invalid_argument("YOLO confidence/NMS thresholds must be in 0..1");
    mc::YoloxDetector detector(model_path,backend,declare_parameter<std::string>("engine_cache",""),"fp16",
      512,320,float(confidence),float(nms),256*1024*1024);
    for(int i=0;i<3;++i) detector.detect(cv::Mat(height,width,CV_8UC3,cv::Scalar(114,114,114)));
    std::unique_ptr<mc::DistanceModel> mlp;
    std::unique_ptr<mc::RgbDistanceModel> rgb_model;
    std::string distance_model_path;
    if(mode!="collect") {
      distance_model_path=declare_parameter<std::string>("distance_model","");
      if(mode=="rgb_infer" || mode=="rgb_compare") rgb_model=std::make_unique<mc::RgbDistanceModel>(distance_model_path);
      else mlp=std::make_unique<mc::DistanceModel>(distance_model_path);
    }
    if(rgb_model) RCLCPP_INFO(get_logger(),"%s / %s distance inference; StereoDepth=%s; Point Cloud disabled",
      stereo_enabled ? "RGB + Stereo comparison":"RGB-only",rgb_model->algorithm().c_str(),stereo_enabled ? "ON":"OFF");
    auto device=std::make_shared<dai::Device>(dai::UsbSpeed::SUPER);
    std::vector<int> sockets;
    for(const auto& c:device->getConnectedCameraFeatures()) {
      sockets.push_back(int(c.socket));
      RCLCPP_INFO(get_logger(),"Connected camera socket=%d sensor=%s",int(c.socket),c.sensorName.c_str());
    }
    for(auto s:{dai::CameraBoardSocket::CAM_A,dai::CameraBoardSocket::CAM_B,dai::CameraBoardSocket::CAM_C})
      if((stereo_enabled || s==dai::CameraBoardSocket::CAM_A) && std::find(sockets.begin(),sockets.end(),int(s))==sockets.end())
        throw std::runtime_error("Required CAM_A RGB / CAM_B,C stereo socket missing");
    auto calibration=device->getCalibration();
    dai::Pipeline pipeline(device);
    pipeline.setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
    auto camera=pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto output=camera->requestOutput({std::uint32_t(width),std::uint32_t(height)},dai::ImgFrame::Type::BGR888i,
      dai::ImgResizeMode::CROP,float(fps),true);
    auto rgbq=output->createOutputQueue(4,false);
    std::shared_ptr<dai::MessageQueue> depthq;
    point_cloud::Config config; config.fps=fps;
    config.resolution=declare_parameter<std::string>("stereo_resolution","400p");
    config.subpixel=declare_parameter<bool>("stereo_subpixel",false);
    config.dot_intensity=declare_parameter<double>("stereo_ir_dot",.5);
    if(stereo_enabled) depthq=point_cloud::addStereoDepth(pipeline,config)->depth.createOutputQueue(8,false);
    pipeline.build(); pipeline.start();
    if(stereo_enabled && !device->setIrLaserDotProjectorIntensity(float(config.dot_intensity)))
      throw std::runtime_error("Pro dot projector unavailable; check device model");
    auto publisher=create_publisher<std_msgs::msg::String>("~/objects",rclcpp::SensorDataQoS());
    std::filesystem::path root=directory;
    if(mode=="collect") {
      root/=sequence;
      if(!std::filesystem::create_directories(root)) throw std::runtime_error("Sequence already exists; use a new sequence ID");
      std::filesystem::create_directory(root/"images");
    } else std::filesystem::create_directories(root);
    std::ofstream samples;
    if(mode=="collect") samples.open(root/"samples.jsonl");
    auto metrics_path=root/(mode+"_metrics_"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count())+".jsonl");
    std::ofstream metrics(metrics_path);
    if(!metrics || (mode=="collect" && !samples)) throw std::runtime_error("Cannot open output files");
    std::deque<std::shared_ptr<dai::ImgFrame>> rgb_buffer,depth_buffer;
    const auto started=Clock::now(); std::size_t frames=0; std::string contract;
    const std::array<std::string,4> names{"con","truck","sedan","traffic_light"};
    if(preview) {
      cv::namedWindow("YOLO distance",cv::WINDOW_NORMAL);
      if(rgb_model)cv::resizeWindow("YOLO distance",1280,720);
    }
    while(rclcpp::ok() && pipeline.isRunning()) {
      rclcpp::spin_some(shared_from_this());
      while(auto d=depthq ? depthq->tryGet<dai::ImgFrame>():nullptr) {depth_buffer.push_back(d); if(depth_buffer.size()>16) depth_buffer.pop_front();}
      while(auto r=rgbq->tryGet<dai::ImgFrame>()) {rgb_buffer.push_back(r); if(rgb_buffer.size()>4) rgb_buffer.pop_front();}
      if(rgb_buffer.empty()) {std::this_thread::sleep_for(std::chrono::milliseconds(1));continue;}
      auto rgb=rgb_buffer.front();
      auto captured=rgb->getTimestamp(dai::CameraExposureOffset::MIDDLE);
      std::shared_ptr<dai::ImgFrame> depth; double skew=1e9;
      if(stereo_enabled) {
        for(auto d:depth_buffer) {
          double delta=std::abs(seconds(d->getTimestamp(dai::CameraExposureOffset::MIDDLE))-seconds(captured))*1000;
          if(delta<skew) {skew=delta;depth=d;}
        }
        // Wait for a depth frame on/after RGB, bounded by 150 ms. Never pair by arrival order.
        if((depth_buffer.empty() || depth_buffer.back()->getTimestamp(dai::CameraExposureOffset::MIDDLE)<captured) && elapsed(captured)<150) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue;
        }
        if(skew>sync_ms) depth.reset();
      }
      rgb_buffer.pop_front();
      if(elapsed(captured)>500) continue;
      auto start=Clock::now(); auto image=rgb->getCvFrame();
      if(image.cols!=width || image.rows!=height || image.type()!=CV_8UC3)
        throw std::runtime_error("Unexpected RGB frame dimensions/type");
      auto rk=intrinsics(*rgb);
      mc::Json geometry={{"device",device->getDeviceId()},{"width",width},{"height",height},
        {"intrinsics",{rk.fx,rk.fy,rk.cx,rk.cy}},{"undistorted",true},{"resize","CROP"}};
      const auto current_contract=geometry.dump();
      if(contract.empty()) {
        contract=current_contract; if(mlp) mlp->checkCamera(contract);
        if(rgb_model) rgb_model->checkCamera(contract);
        mc::Json metadata={{"camera",geometry},{"sequence",sequence},{"mode",mode},{"distance_definition","rgb_optical_z_m"},
          {"sync_ms",sync_ms},{"core_fraction",quality.core},{"min_valid_ratio",quality.min_ratio},
          {"max_relative_spread",quality.max_relative_spread},{"min_pixels",quality.min_pixels},
          {"min_depth_m",quality.min_m},{"max_depth_m",quality.max_m},{"yolo_model",model_path}};
        metadata["stereo_enabled"]=stereo_enabled;
        metadata["distance_model"]=distance_model_path;
        metadata["distance_algorithm"]=rgb_model ? rgb_model->algorithm() : (mlp ? "legacy_mlp":"stereo");
        metadata["confidence_threshold"]=confidence; metadata["nms_threshold"]=nms;
        metadata["stereo_resolution"]=config.resolution;metadata["stereo_subpixel"]=config.subpixel;
        std::ofstream meta(root/(mode+"_metadata.json")); meta<<metadata.dump(2);
      } else if(contract!=current_contract) throw std::runtime_error("Frame geometry changed");
      auto yolo_start=Clock::now(); auto detections=detector.detect(image).detections; double yolo_ms=elapsed(yolo_start);
      std::vector<std::optional<double>> predictions;
      std::vector<std::string> prediction_reasons;
      auto mlp_start=Clock::now();
      for(const auto& d:detections) {
        if(rgb_model) {
          const auto p=rgb_model->predict(d.box.x,d.box.y,d.box.width,d.box.height,width,height,d.class_id);
          predictions.push_back(p.distance);prediction_reasons.push_back(p.reason);
        } else {
          predictions.push_back(mlp ? mlp->predict(mc::distanceFeatures(d.box.x,d.box.y,
            d.box.width,d.box.height,width,height,d.class_id),d.class_id):std::optional<double>{});
          prediction_reasons.emplace_back();
        }
      }
      double mlp_ms=elapsed(mlp_start);
      auto depth_alignment_start=Clock::now();
      cv::Mat aligned; if(depth && !detections.empty()) aligned=alignDepth(*rgb,*depth,calibration);
      const double depth_alignment_ms=elapsed(depth_alignment_start);
      mc::Json objects=mc::Json::array();
      const auto image_name="images/"+std::to_string(rgb->getSequenceNum())+".png";
      bool saved=false;
      for(std::size_t i=0;i<detections.size();++i) {
        const auto& d=detections[i]; auto b=d.box;
        std::optional<double> label;
        mc::DepthQuality::Measurement measurement;
        measurement.reason=stereo_enabled ? "no_synchronized_depth":"disabled";
        if(!aligned.empty()) {
          cv::Rect core(int(std::ceil(b.x+b.width*(1-quality.core)/2)),int(std::ceil(b.y+b.height*(1-quality.core)/2)),
            std::max(1,int(b.width*quality.core)),std::max(1,int(b.height*quality.core)));
          core&=cv::Rect(0,0,width,height);
          std::vector<double> valid;
          for(int y=core.y;y<core.y+core.height;++y) for(int x=core.x;x<core.x+core.width;++x) {
            double z=aligned.at<double>(y,x); if(z>=quality.min_m && z<=quality.max_m) valid.push_back(z);
          }
          measurement=quality.assess(valid,core.area());label=measurement.distance;
        }
        mc::Json obj={{"class_id",d.class_id},{"class",names.at(d.class_id)},{"confidence",d.score},
          {"bbox",{b.x,b.y,b.width,b.height}},{"distance_m",nullptr},{"stereo_distance_m",nullptr}};
        if(predictions[i]) obj["distance_m"]=*predictions[i];
        if(rgb_model) {
          obj["distance_source"]=rgb_model->algorithm(); obj["prediction_reason"]=prediction_reasons[i];
        }
        if(label) obj["stereo_distance_m"]=*label;
        obj["stereo_status"]=label ? "valid":(stereo_enabled ? "unavailable":"disabled");
        obj["stereo_reason"]=measurement.reason;
        obj["stereo_roi_pixels"]=measurement.roi_pixels;obj["stereo_valid_pixels"]=measurement.valid_pixels;
        obj["stereo_valid_ratio"]=measurement.valid_ratio;obj["stereo_relative_spread"]=measurement.relative_spread;
        obj["prediction_minus_stereo_m"]=nullptr;
        if(predictions[i] && label)obj["prediction_minus_stereo_m"]=*predictions[i]-*label;
        if(mode=="collect" && label) {
          if(!saved) {if(!cv::imwrite((root/image_name).string(),image)) throw std::runtime_error("Image write failed");saved=true;}
          auto row=obj; row["distance_m"]=*label;
          row["sequence"]=sequence; row["image"]=image_name; row["width"]=width; row["height"]=height;
          row["timestamp"]=seconds(captured); row["depth_timestamp"]=seconds(depth->getTimestamp(dai::CameraExposureOffset::MIDDLE));
          row["camera_contract"]=contract; samples<<row.dump()<<'\n';
        }
        objects.push_back(obj);
      }
      ++frames;
      mc::Json record={{"timestamp",seconds(captured)},{"timestamp_clock","depthai_host_steady"},
        {"frame_id","oak_rgb_optical_frame"},{"sequence_number",rgb->getSequenceNum()},
        {"objects",objects},{"yolo_ms",yolo_ms},{"mlp_ms",mlp_ms},{"yolo_mlp_ms",yolo_ms+mlp_ms},
        {"processing_ms",elapsed(start)},{"capture_to_result_ms",elapsed(captured)},
        {"fps",frames/(seconds(Clock::now())-seconds(started))},{"sync_ms",depth ? mc::Json(skew):mc::Json(nullptr)}};
      record["stereo_enabled"]=stereo_enabled;
      record["distance_model_ms"]=mlp_ms;
      record["depth_alignment_ms"]=depth_alignment_ms;
      record["yolo_distance_ms"]=yolo_ms+mlp_ms;
      record["distance_algorithm"]=rgb_model ? rgb_model->algorithm() : (mlp ? "legacy_mlp":"stereo");
      std_msgs::msg::String message; message.data=record.dump();publisher->publish(message);
      metrics<<message.data<<'\n'; metrics.flush(); if(samples.is_open()) samples.flush();
      if(!metrics || (samples.is_open() && !samples)) throw std::runtime_error("Output write failed");
      if(preview) {
        cv::Mat display;
        if(rgb_model) display=mc::liveDistancePreview(image,objects,rgb_model->algorithm(),stereo_enabled,
          record["fps"].get<double>(),yolo_ms,mlp_ms);
        else {
          display=image;
          for(std::size_t i=0;i<detections.size();++i) {
            const auto& d=detections[i]; auto value=mode=="collect" ? objects[i]["stereo_distance_m"]:objects[i]["distance_m"];
            std::string text=names[d.class_id]+cv::format(" / %.2f / ",d.score)+
              (value.is_null() ? "N/A":cv::format("%.2f m",value.get<double>()));
            cv::rectangle(display,d.box,{0,255,0},2);
            cv::putText(display,text,{int(d.box.x),std::max(15,int(d.box.y)-5)},cv::FONT_HERSHEY_SIMPLEX,.5,{0,255,0},1);
          }
        }
        cv::imshow("YOLO distance",display); int key=cv::waitKey(1);if(key==27 || key=='q') break;
      }
    }
    pipeline.stop();
  }
};
int main(int argc,char**argv) {
  rclcpp::init(argc,argv); int status=0;
  try {std::make_shared<DistanceNode>()->run();}
  catch(const std::exception& e) {RCLCPP_ERROR(rclcpp::get_logger("yolo_distance"),"%s",e.what());status=1;}
  rclcpp::shutdown();return status;
}
