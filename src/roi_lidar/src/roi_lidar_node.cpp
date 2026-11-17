#include "roi_lidar/config.hpp"
#include "roi_lidar/preview.hpp"
#include "roi_lidar/freshness.hpp"
#include "roi_lidar/controls.hpp"
#include <point_cloud/startup_parameters.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <exception>
#include <mutex>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;
namespace roi_lidar
{
using Clock=std::chrono::steady_clock;
static double seconds(Clock::time_point t) {return std::chrono::duration<double>(t.time_since_epoch()).count();}
static point_cloud::Intrinsics intrinsics(dai::ImgFrame & frame)
{
  const auto & metadata=frame.getTransformation();
  if (!metadata.isValid()) {throw std::runtime_error("Missing frame intrinsics");}
  const auto k=metadata.getIntrinsicMatrix();
  return {k[0][0],k[1][1],k[0][2],k[1][2]};
}
static std::vector<double> geometryKey(dai::ImgFrame & frame, const point_cloud::RigidTransform & t)
{
  const auto k=intrinsics(frame);
  std::vector<double> key{double(frame.getWidth()),double(frame.getHeight()),k.fx,k.fy,k.cx,k.cy};
  key.insert(key.end(),t.rotation.begin(),t.rotation.end());
  key.insert(key.end(),t.translation.begin(),t.translation.end()); return key;
}
static const char * groundModel(const Options & options, const Scan & scan)
{
  if (scan.ground_plane.detected) {return "RANSAC";}
  if (options.ransac.enabled) {return "FALLBACK";}
  return options.use_startup_plane ? "STARTUP" : "FIXED";
}
static cv::Mat frameView(dai::ImgFrame & frame, int cv_type)
{
  const auto type=frame.getType();
  if ((cv_type==CV_16UC1 && type!=dai::ImgFrame::Type::RAW16) ||
    (cv_type==CV_8UC3 && type!=dai::ImgFrame::Type::BGR888i) ||
    (cv_type==CV_8UC1 && type!=dai::ImgFrame::Type::RAW8 &&
     type!=dai::ImgFrame::Type::GRAY8 && type!=dai::ImgFrame::Type::YUV400p)) {
    throw std::runtime_error("Unexpected preview pixel format");
  }
  const auto width=frame.getWidth(), height=frame.getHeight();
  const std::size_t row=width*CV_ELEM_SIZE(cv_type);
  const std::size_t stride=frame.getStride() ? frame.getStride() : row;
  auto && bytes=frame.getData();
  if (!width || !height || stride<row || bytes.size()<row ||
    (height>1 && stride>(bytes.size()-row)/(height-1))) {throw std::runtime_error("Truncated image frame");}
  return cv::Mat(height,width,cv_type,bytes.data(),stride);
}
static void apply(Config & c,const rclcpp::Parameter & p)
{
  if (p.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER &&
    (p.as_int()<std::numeric_limits<int>::min() || p.as_int()>std::numeric_limits<int>::max())) {
    throw std::invalid_argument("Integer out of range");
  }
#define APPLY(name,field,getter) if (p.get_name()==name) {c.field=p.getter(); return;}
  ROI_PARAMETERS(APPLY)
#undef APPLY
}
// Joins before the local DepthSource is destroyed, including exception/reopen paths.
class ScopedSensorReader
{
  std::atomic_bool done_{false};
  std::mutex mutex_;
  std::exception_ptr error_;
  std::thread thread_;
public:
  template<class Function> ScopedSensorReader(bool enabled, Function function) {
    if(enabled)thread_=std::thread([this,function] {
      try {function(done_);}
      catch(...) {std::lock_guard<std::mutex> lock(mutex_);error_=std::current_exception();}
    });
  }
  ~ScopedSensorReader() {done_=true;if(thread_.joinable())thread_.join();}
  void rethrowIfFailed() {
    std::lock_guard<std::mutex> lock(mutex_);if(error_)std::rethrow_exception(error_);
  }
};
class Node : public rclcpp::Node
{
  struct Snapshot
  {
    Scan scan;
    Clusters clusters;
    Terrain terrain;
    std::shared_ptr<dai::ImgFrame> depth, right, rgb;
    point_cloud::RigidTransform vehicle_from_rgb;
    std::uint64_t revision{0};
    double capture{0}, fps{0}, host_ms{0}, age_ms{0};
    std::string state{"MEASURING GROUND: keep stationary"};
  };
  std::atomic_bool parameters_locked_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr running_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr locked_pub_;
  Config config_;
  Snapshot snapshot_;
  std::mutex mutex_;
  std::atomic_bool stop_{false};
  std::atomic<std::uint64_t> camera_revision_{0};
  std::uint64_t revision_{0};
  std::thread worker_;
  oak_startup::OakStartupMeasurementConfig startup_;
  cv::Mat startup_preview_;
  std::atomic_bool startup_finished_{false};
  bool startup_gui_requested_{false},startup_window_{false};
  static constexpr const char* startupWindowName(){return "OAK startup ground-plane ROI";}
  std::string frame_id_,device_id_;
  double camera_x_,camera_y_,camera_yaw_;
  std::vector<std::string> names_;
  std::vector<rclcpp::Parameter> last_parameters_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_pub_, raw_terrain_pub_, planning_terrain_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_pub_, roi_pub_, lane_bev_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;
  rclcpp::TimerBase::SharedPtr sync_timer_, preview_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_;
  BevProjector bev_, lane_bev_;
  Controls controls_;
  std::vector<double> rgb_key_, lane_rgb_key_;
  double last_lane_capture_{-1};
  std::uint64_t preview_revision_{std::numeric_limits<std::uint64_t>::max()};
  double last_preview_{0};
  bool bev_window_{false};
  bool windows_{false}, gui_failed_{false}, dragging_{false};
  int roi_width_{0},roi_height_{0};
  cv::Point drag_start_;
  bool stopping() const {return stop_ || !rclcpp::ok();}
  void state(const std::string & text)
  {
    {std::lock_guard<std::mutex> lock(mutex_); snapshot_.state=text;}
    std_msgs::msg::String msg; msg.data=text; status_pub_->publish(msg);
  }
  Config readParameters()
  {
    Config c; for (const auto & p:get_parameters(names_)) {apply(c,p);} validate(c); return c;
  }
  void syncParameters()
  {
    const auto params=get_parameters(names_);
    bool changed=false,reopen=false;
    for (std::size_t i=0;i<params.size();++i) {
      if (params[i].to_parameter_msg()==last_parameters_[i].to_parameter_msg()) {continue;}
      changed=true;
      const auto & n=names_[i];
      reopen|=n.rfind("camera.",0)==0 || n.rfind("depth.",0)==0 ||
        n.rfind("motion.",0)==0 || n=="preview.publish_lane_bev" || n.rfind("preview.rgb_",0)==0 || n=="preview.camera_image" || n=="preview.gui" || n=="preview.publish";
    }
    if (!changed) {return;}
    auto c=readParameters();
    {std::lock_guard<std::mutex> lock(mutex_); config_=c; ++revision_;}
    if (reopen) {++camera_revision_;}
    last_parameters_=params;
  }
  void publishScan(const Scan & scan,const Config & c,const rclcpp::Time & stamp,double period)
  {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp=stamp; msg.header.frame_id=frame_id_;
    msg.angle_min=c.scan.angle_min*radians; msg.angle_max=c.scan.angle_max*radians;
    msg.angle_increment=(msg.angle_max-msg.angle_min)/(c.scan.bins-1);
    msg.range_min=c.scan.min_range; msg.range_max=c.scan.max_range;
    msg.scan_time=std::max(0.0,period); msg.time_increment=0; msg.ranges=scan.ranges;
    scan_pub_->publish(msg);
  }
  void publishObstacles(const Clusters & clusters,const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseArray msg; msg.header.stamp=stamp; msg.header.frame_id=frame_id_;
    for (const auto & object:clusters.objects) {
      geometry_msgs::msg::Pose pose;
      pose.position.x=object.x; pose.position.y=object.y; pose.position.z=0;
      pose.orientation.w=1.0; msg.poses.push_back(pose);
    }
    obstacles_pub_->publish(msg);
  }
  void publishTerrain(const Terrain & terrain, const Scan & scan, const Config & c,
    const rclcpp::Time & stamp)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp=stamp; grid.header.frame_id=frame_id_;
    grid.info.resolution=terrain.cell_m; grid.info.width=terrain.width; grid.info.height=terrain.height;
    grid.info.origin.position.y=terrain.y_min; grid.info.origin.orientation.w=1;
    grid.data=terrain.cells; terrain_pub_->publish(grid);
    grid.data=terrain.raw_cells; raw_terrain_pub_->publish(grid);
    grid.data=terrain.planning_cells; planning_terrain_pub_->publish(grid);
    if (points_pub_->get_subscription_count() == 0) {return;}
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header=grid.header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(5,"x",1,sensor_msgs::msg::PointField::FLOAT32,
      "y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32,
      "height",1,sensor_msgs::msg::PointField::FLOAT32,"rgb",1,sensor_msgs::msg::PointField::FLOAT32);
    const std::size_t count=scan.processed ? scan.obstacle_indices.size() : std::count_if(scan.surface_points.begin(),scan.surface_points.end(),
      [&](const auto & p) {return projectedObstacle(p,c.scan,&scan);});
    modifier.resize(count); cloud.is_dense=true;
    if (count>0) {
      sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z"),height(cloud,"height");
      sensor_msgs::PointCloud2Iterator<float> color(cloud,"rgb");
      const std::uint32_t packed_red=0x00ff0000;
      float red; std::memcpy(&red,&packed_red,sizeof(red));
      forEachObstacle(scan,c.scan,[&](const auto& p){
        *x=p[0]; *y=p[1]; *z=0; *height=p[2]; *color=red;
        ++x; ++y; ++z; ++height; ++color;
      });
    }
    points_pub_->publish(cloud);
  }
  void invalidate(const Config & c,const std::string & text)
  {
    {std::lock_guard<std::mutex> lock(mutex_); snapshot_=Snapshot{}; snapshot_.state=text;}
    const auto empty=emptyScan(c.scan);
    publishScan(empty,c,now(),0); publishObstacles(Clusters{},now());
    publishTerrain(terrainGrid(empty,c.scan,c.view,c.terrain),empty,c,now()); state(text);
  }
  void run()
  {
    point_cloud::RigidTransform vehicle_from_camera;
    std::string selected=device_id_;
    try {
      state("MEASURING GROUND: keep stationary on a flat floor");
      const auto pose=oak_startup::measureOakStartupExtrinsics(startup_,[this](){return stopping();});
      startup_finished_=true;
      {std::lock_guard<std::mutex> lock(mutex_);startup_preview_.release();}
      selected=pose.device_id;
      vehicle_from_camera=point_cloud::vehicleFromRgb(camera_x_,camera_y_,pose.height_m,
        pose.roll_deg,pose.pitch_down_deg,camera_yaw_);
      RCLCPP_INFO(get_logger(),"Ground fixed: height=%.3fm roll=%.2f pitch=%.2f",pose.height_m,pose.roll_deg,pose.pitch_down_deg);
    } catch (const std::exception & e) {
      startup_finished_=true;
      {std::lock_guard<std::mutex> lock(mutex_);startup_preview_.release();}
      state(std::string("STARTUP FAILED: restart after fixing ground ROI: ")+e.what()); return;
    }
    while (!stopping()) {
      Config startup_config;
      {std::lock_guard<std::mutex> lock(mutex_); startup_config=config_;}
      const auto camera_revision=camera_revision_.load();
      try {
        invalidate(startup_config,"OPENING CAMERA");
        point_cloud::DepthSource source(startup_config.camera,selected,
          startup_config.camera_image && (startup_config.gui || startup_config.publish_preview),
          (startup_config.gui || startup_config.publish_preview || startup_config.publish_lane_bev) ? startup_config.rgb_fps : 0.0,startup_config.publish_imu);
        // rgbFromFrame updates a calibration cache, shared by depth/RGB readers.
        std::mutex geometry_mutex;
        auto transformFrame=[&](dai::ImgFrame& frame) {
          std::lock_guard<std::mutex> lock(geometry_mutex);
          return point_cloud::compose(vehicle_from_camera,source.rgbFromFrame(frame));
        };
        // Match camera_driver's independent IMU reader: depth RANSAC/outlier
        // work must not throttle the 400Hz IMU or leave old batches queued.
        ScopedSensorReader imu_reader(startup_config.publish_imu,[&](const std::atomic_bool& done) {
          while(!done && !stopping() && camera_revision==camera_revision_) {
            bool received=false;
            for(int batch=0;batch<startup_config.camera.imu_queue_size && !done && !stopping();++batch) {
              auto data=source.tryGetImu();if(!data)break;received=true;
              const auto host_now=Clock::now();const auto ros_now=now();
              double maximum_age;
              {std::lock_guard<std::mutex> lock(mutex_);maximum_age=config_.max_age;}
              for(const auto& packet:data->packets) {
                const auto& gyro=packet.gyroscope;
                const double age=seconds(host_now)-seconds(gyro.getTimestamp());
                if(!std::isfinite(age) || age<-.01 || age>maximum_age)continue;
                const auto rate=point_cloud::rotate(vehicle_from_camera.rotation,
                  source.cameraGyroscope({gyro.x,gyro.y,gyro.z}));
                if(!std::isfinite(rate[0]) || !std::isfinite(rate[1]) || !std::isfinite(rate[2]))continue;
                sensor_msgs::msg::Imu msg;
                msg.header.stamp=ros_now-rclcpp::Duration::from_seconds(age);msg.header.frame_id=frame_id_;
                msg.orientation_covariance[0]=-1;msg.linear_acceleration_covariance[0]=-1;
                msg.angular_velocity.x=rate[0];msg.angular_velocity.y=rate[1];msg.angular_velocity.z=rate[2];
                imu_pub_->publish(msg);
              }
            }
            if(!received)std::this_thread::sleep_for(1ms);
          }
        });
        // RGB must not wait for the slower depth/terrain processing loop.
        ScopedSensorReader rgb_reader(startup_config.gui || startup_config.publish_preview || startup_config.publish_lane_bev,
          [&](const std::atomic_bool& done) {
            while(!done && !stopping() && camera_revision==camera_revision_) {
              if(auto rgb=source.tryGetRgb()) {
                const auto transform=transformFrame(*rgb);
                std::lock_guard<std::mutex> lock(mutex_);
                snapshot_.rgb=std::move(rgb);snapshot_.vehicle_from_rgb=transform;
              } else {std::this_thread::sleep_for(1ms);}
            }
          });
        Projector projector;
        std::vector<double> key;
        std::uint64_t configured_revision=std::numeric_limits<std::uint64_t>::max();
        double last_capture=0,last_rx=seconds(Clock::now()),report=last_rx;
        std::size_t frames=0; double work_ms=0,fps=0,host_ms=0;
        double scan_ms=0, terrain_ms=0;
        bool stale=false;
        while (!stopping() && camera_revision==camera_revision_) {
          imu_reader.rethrowIfFailed();rgb_reader.rethrowIfFailed();
          Config c; std::uint64_t version;
          {std::lock_guard<std::mutex> lock(mutex_); c=config_; version=revision_;}
          const auto host=Clock::now(); const double clock=seconds(host);
          if (auto right=source.tryGetRight()) {
            std::lock_guard<std::mutex> lock(mutex_); snapshot_.right=std::move(right);
          }
          if (clock-last_rx>c.max_age && !stale) {invalidate(c,"STALE: no fresh depth"); stale=true;}
          auto depth=source.tryGet();
          if (!depth) {std::this_thread::sleep_for(1ms); continue;}
          const double capture=seconds(depth->getTimestamp()),age=clock-capture;
          if (age<-.01 || age>c.max_age || capture<=last_capture) {continue;}
          if (depth->getType()!=dai::ImgFrame::Type::RAW16) {throw std::runtime_error("Expected RAW16 depth");}
          const auto transform=transformFrame(*depth);
          const auto next_key=geometryKey(*depth,transform);
          if (key!=next_key || configured_revision!=version) {
            projector.configure(depth->getWidth(),depth->getHeight(),intrinsics(*depth),transform,c.scan);
            key=next_key; configured_revision=version;
          }
          const auto & bytes=depth->getData();
          const auto scan_begin=Clock::now();
          auto scan=projector.project(bytes.data(),bytes.size(),depth->getStride() ? depth->getStride() : depth->getWidth()*2,c.terrain.below_ground_m);
          const auto scan_end=Clock::now();
          auto clusters=clusterScan(scan,c.scan,c.cluster);
          auto terrain=terrainGrid(scan,c.scan,c.view,c.terrain);
          const auto terrain_end=Clock::now();
          const double publish_age=seconds(terrain_end)-capture;
          if(publish_age>c.max_age){invalidate(c,"STALE: filtering exceeded input age");continue;}
          auto stamp=now();
          const auto delay=rclcpp::Duration::from_seconds(std::max(0.0,publish_age));
          if (stamp.nanoseconds()>=delay.nanoseconds()) {stamp=stamp-delay;}
          // Never publish an in-flight result after settings were replaced.
          {std::lock_guard<std::mutex> lock(mutex_); if (version!=revision_) {continue;}}
          publishScan(scan,c,stamp,last_capture>0 ? capture-last_capture : 0);
          publishObstacles(clusters,stamp);
          publishTerrain(terrain,scan,c,stamp);
          scan_ms+=std::chrono::duration<double,std::milli>(scan_end-scan_begin).count();
          terrain_ms+=std::chrono::duration<double,std::milli>(terrain_end-scan_end).count();
          ++frames; work_ms+=std::chrono::duration<double,std::milli>(Clock::now()-host).count();
          if (clock-report>=1) {
            fps=frames/(clock-report); host_ms=work_ms/frames;
            RCLCPP_INFO(get_logger(),"requested=%.1f depth_fps=%.1f host=%.2fms scan=%.2fms grid_cluster=%.2fms age=%.1fms ROI_rays=%zu depth=%zu ground_removed=%zu height_removed=%zu accepted=%zu bins=%zu floor_cells=%zu hazard_cells=%zu clusters=%zu range_limit=%.1fm ground_model=%s inliers=%zu/%zu outlier_removed=%zu/%zu",
              c.camera.fps,fps,host_ms,scan_ms/frames,terrain_ms/frames,publish_age*1000,projector.rayCount(),scan.depth_points,scan.ground_removed,scan.height_removed,scan.accepted_points,scan.valid_bins,terrain.ground_cells,terrain.obstacle_cells,clusters.objects.size(),c.scan.max_range,groundModel(c.scan,scan),scan.ground_plane.inliers,scan.ground_plane.samples,scan.outlier_removed,scan.outlier_candidates);
            std_msgs::msg::Float64MultiArray timing;
            // Milliseconds per processed depth frame, followed by effective FPS.
            timing.data={host_ms,scan_ms/frames,terrain_ms/frames,fps};
            timing_pub_->publish(timing);
            report=clock; frames=0; work_ms=scan_ms=terrain_ms=0;
          }
          {std::lock_guard<std::mutex> lock(mutex_);
            snapshot_.scan=std::move(scan); snapshot_.clusters=std::move(clusters); snapshot_.terrain=std::move(terrain);
            snapshot_.depth=std::move(depth); snapshot_.capture=capture;
            snapshot_.revision=version; snapshot_.fps=fps; snapshot_.host_ms=host_ms; snapshot_.age_ms=publish_age*1000;
            snapshot_.state="STREAMING";
          }
          if (last_capture==0 || stale) {state("STREAMING");}
          last_capture=capture; last_rx=clock; stale=false;
        }
      } catch (const std::exception & e) {
        invalidate(startup_config,std::string("CAMERA ERROR: ")+e.what());
        RCLCPP_ERROR(get_logger(),"%s",e.what());
        for (int i=0;i<20 && !stopping() && camera_revision==camera_revision_;++i) {std::this_thread::sleep_for(100ms);}
      }
    }
  }
  sensor_msgs::msg::Image imageMessage(const cv::Mat & image,const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::Image msg; msg.header.stamp=stamp; msg.header.frame_id=frame_id_;
    msg.width=image.cols; msg.height=image.rows; msg.encoding="bgr8"; msg.step=image.cols*3;
    msg.data.resize(msg.step*msg.height);
    for (int row=0;row<image.rows;++row) {std::memcpy(msg.data.data()+row*msg.step,image.ptr(row),msg.step);}
    return msg;
  }
  static void mouse(int event,int x,int y,int,void * context)
  {
    auto & self=*static_cast<Node *>(context);
    if (event==cv::EVENT_LBUTTONDOWN) {self.dragging_=true; self.drag_start_={x,y};}
    if (event!=cv::EVENT_LBUTTONUP || !self.dragging_) {return;}
    self.dragging_=false;
    if (self.roi_width_<1 || self.roi_height_<1) {return;}
    const int x0=std::clamp(std::min(x,self.drag_start_.x),0,self.roi_width_-1);
    const int y0=std::clamp(std::min(y,self.drag_start_.y),0,self.roi_height_-1);
    const int x1=std::clamp(std::max(x,self.drag_start_.x),x0+1,self.roi_width_);
    const int y1=std::clamp(std::max(y,self.drag_start_.y),y0+1,self.roi_height_);
    if (x1-x0<4 || y1-y0<4) {return;}
    const auto result=self.set_parameters_atomically({
      rclcpp::Parameter("roi.x",double(x0)/self.roi_width_),rclcpp::Parameter("roi.y",double(y0)/self.roi_height_),
      rclcpp::Parameter("roi.width",double(x1-x0)/self.roi_width_),rclcpp::Parameter("roi.height",double(y1-y0)/self.roi_height_)});
    if (!result.successful) {RCLCPP_WARN(self.get_logger(),"ROI: %s",result.reason.c_str());}
  }
  void preview()
  {
    Config c; Snapshot s; std::uint64_t version;
    {std::lock_guard<std::mutex> lock(mutex_); c=config_; version=revision_;}
    const double clock=seconds(Clock::now());
    if (preview_revision_!=version) {
      rgb_key_.clear();lane_rgb_key_.clear();last_lane_capture_=-1;preview_revision_=version;
    }
    // Lane input is published once per fresh RGB exposure, independently of GUI FPS.
    if(startup_finished_ && c.publish_lane_bev) {
      Snapshot rgb;
      {std::lock_guard<std::mutex> lock(mutex_);rgb.rgb=snapshot_.rgb;rgb.vehicle_from_rgb=snapshot_.vehicle_from_rgb;}
      if(rgb.rgb) {
        const double captured=seconds(rgb.rgb->getTimestamp());
        if(captured!=last_lane_capture_ && freshCapture(captured,clock,c.max_age)) {
          try {
            const auto key=geometryKey(*rgb.rgb,rgb.vehicle_from_rgb);
            if(key!=lane_rgb_key_) {
              lane_bev_.configureLane(rgb.rgb->getWidth(),rgb.rgb->getHeight(),intrinsics(*rgb.rgb),rgb.vehicle_from_rgb);
              lane_rgb_key_=key;
            }
            const auto stamp=now()-rclcpp::Duration::from_seconds(std::max(0.0,clock-captured));
            lane_bev_pub_->publish(imageMessage(lane_bev_.render(frameView(*rgb.rgb,CV_8UC3)),stamp));
            last_lane_capture_=captured;
          } catch(const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),1000,"Lane BEV: %s",e.what());
          }
        }
      }
    }
    if (clock-last_preview_<1.0/c.preview_fps) {return;} last_preview_=clock;
    if (startup_finished_ && startup_window_) {
      try {cv::destroyWindow(startupWindowName());cv::waitKey(1);}catch(const cv::Exception&){}
      startup_window_=false;
    }
    if (!startup_finished_) {
      // The measurement worker only supplies images. All HighGUI stays on the
      // ROS/main thread, using the exact ROI overlay generated by oak_startup.
      cv::Mat image;
      {std::lock_guard<std::mutex> lock(mutex_);image=startup_preview_;}
      if(startup_gui_requested_ && c.gui && !gui_failed_ && !image.empty()) {
        try {
          if(!startup_window_) {
            cv::namedWindow(startupWindowName(),cv::WINDOW_NORMAL);
            cv::resizeWindow(startupWindowName(),image.cols,image.rows);startup_window_=true;
          }
          cv::imshow(startupWindowName(),image);
          const int key=cv::waitKey(1);
          if(key=='q' || key==27){stop_=true;rclcpp::shutdown();}
        }catch(const cv::Exception& e){
          gui_failed_=true;RCLCPP_ERROR(get_logger(),"Startup preview: %s",e.what());
        }
      }
      return;
    }
    if (!c.gui && windows_) {
      try {cv::destroyAllWindows();} catch (const cv::Exception &) {}
      windows_=false;bev_window_=false;
      controls_.reset();
    }
    const bool publish_bev=c.publish_preview && preview_pub_->get_subscription_count()>0;
    const bool publish_roi=c.publish_preview && roi_pub_->get_subscription_count()>0;
    const bool render_bev=publish_bev || (c.gui && !gui_failed_ && c.show_bev_window);
    if (!c.gui && !publish_bev && !publish_roi && !c.publish_lane_bev) {return;}
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (render_bev) {
        s=snapshot_;
      } else {
        s.depth=snapshot_.depth;
        s.right=snapshot_.right;
        s.rgb=snapshot_.rgb;
        s.vehicle_from_rgb=snapshot_.vehicle_from_rgb;
        s.capture=snapshot_.capture;
        s.revision=snapshot_.revision;
      }
    }
    try {
      const bool fresh=s.depth && s.revision==version && freshCapture(s.capture,clock,c.max_age);
      const bool rgb_fresh=s.rgb && freshCapture(seconds(s.rgb->getTimestamp()),clock,c.max_age);
      const bool aligned=fresh && rgb_fresh && overlayAllowed(s.capture,seconds(s.rgb->getTimestamp()),clock,c.max_age,c.sync_sec);
      cv::Mat background;
      if (rgb_fresh && render_bev) {
        const auto key=geometryKey(*s.rgb,s.vehicle_from_rgb);
        if (key!=rgb_key_) {
          bev_.configure(s.rgb->getWidth(),s.rgb->getHeight(),intrinsics(*s.rgb),s.vehicle_from_rgb,c.view,c.preview_size);
          rgb_key_=key;
        }
        background=bev_.render(frameView(*s.rgb,CV_8UC3),c.interpolation);
      }
      cv::Mat image;
      if (render_bev) {
        std::ostringstream text; text<<s.state<<" | depth "<<std::fixed<<std::setprecision(1)<<s.fps
          <<" FPS / host "<<s.host_ms<<" ms | max height "<<c.scan.max_height<<"m"
          <<" | floor "<<(fresh ? s.terrain.ground_cells : 0)<<" hazard "<<(fresh ? s.terrain.obstacle_cells : 0)
          <<" filled "<<(fresh ? s.terrain.filled_cells : 0)
          <<" | plane "<<groundModel(c.scan,s.scan)
          <<" "<<s.scan.ground_plane.inliers<<"/"<<s.scan.ground_plane.samples
          <<" | isolated "<<(fresh ? s.scan.outlier_removed : 0);
        if (!rgb_fresh) {text<<" | WAIT RGB";}
        else if (!aligned) {text<<" | OVERLAY WAIT SYNC";}
        else {text<<" | delta "<<std::abs(s.capture-seconds(s.rgb->getTimestamp()))*1000<<" ms";}
        // With no RGB available, still show fresh depth on a plain metric canvas.
        // With fresh but unsynchronized RGB, suppress the depth overlay.
        const bool visible=fresh && (!rgb_fresh || aligned);
        const auto empty=emptyScan(c.scan);
        const Clusters hidden_scan;
        image=mapPreview(empty,c.scan,c.view,c.preview_size,text.str(),background,&hidden_scan,false);
        const auto unknown=visible ? Terrain{} : terrainGrid(empty,c.scan,c.view,c.terrain);
        terrainOverlay(image,visible ? s.terrain : unknown,visible ? s.scan : empty,c.scan,
          c.view,c.preview_size,c.show_terrain,c.show_projected_points,c.terrain_alpha,c.point_radius,c.terrain.vehicle_width_m);
        if (publish_bev) {preview_pub_->publish(imageMessage(image,now()));}
      }
      cv::Mat roi;
      if (publish_roi || (c.gui && !gui_failed_)) {
        if (fresh) {
          const bool right=c.camera_image && s.right &&
            std::abs(seconds(s.right->getTimestamp())-s.capture)<=c.sync_sec;
          auto frame=right ? s.right : s.depth;
          roi=roiPreview(frameView(*frame,right ? CV_8UC1 : CV_16UC1),c.scan,!right);
        } else {
          roi=cv::Mat(400,640,CV_8UC3,cv::Scalar(30,30,30));
          cv::putText(roi,"NO FRESH DEPTH",{20,40},cv::FONT_HERSHEY_SIMPLEX,.7,{220,220,220},1,cv::LINE_AA);
        }
        if (publish_roi) {roi_pub_->publish(imageMessage(roi,now()));}
      }
      if (c.gui && !gui_failed_) {
        if (!windows_) {
          cv::namedWindow("ROI LiDAR - select depth ROI",cv::WINDOW_AUTOSIZE);
          cv::setMouseCallback("ROI LiDAR - select depth ROI",mouse,this); windows_=true;
        }
        if(c.show_bev_window) {
          if(!bev_window_) {cv::namedWindow("ROI LiDAR - RGB BEV",cv::WINDOW_AUTOSIZE);bev_window_=true;}
          cv::imshow("ROI LiDAR - RGB BEV",image);
        }else if(bev_window_) {
          cv::destroyWindow("ROI LiDAR - RGB BEV");bev_window_=false;
        }
        roi_width_=fresh ? roi.cols : 0; roi_height_=fresh ? roi.rows : 0;
        cv::imshow("ROI LiDAR - select depth ROI",roi);
        const int key=cv::waitKeyEx(1);
        const bool editing=controls_.editing();
        if (!editing && (key=='q' || key==27)) {stop_=true; rclcpp::shutdown();}
        if (!editing && key=='r') {set_parameters_atomically({rclcpp::Parameter("roi.x",0.0),rclcpp::Parameter("roi.y",0.0),
          rclcpp::Parameter("roi.width",1.0),rclcpp::Parameter("roi.height",1.0)});}
        controls_.setLocked(parameters_locked_.load());
        const auto changes=controls_.poll([this](const char * name) {
          const auto p=get_parameter(name);
          if (p.get_type()==rclcpp::ParameterType::PARAMETER_BOOL) {return p.as_bool() ? 1.0 : 0.0;}
          if (p.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER) {return double(p.as_int());}
          return p.as_double();
        },key);
        if (!changes.empty()) {
          std::vector<rclcpp::Parameter> edits;
          for (const auto & change:changes) {
            if (change.type==Controls::Type::Boolean) {edits.emplace_back(change.name,change.value!=0);}
            else if (change.type==Controls::Type::Integer) {edits.emplace_back(change.name,static_cast<int>(std::lround(change.value)));}
            else {edits.emplace_back(change.name,change.value);}
          }
          const auto result=set_parameters_atomically(edits);
          if (!result.successful) {controls_.message("REJECTED: "+result.reason);
            RCLCPP_WARN(get_logger(),"GUI settings rejected: %s",result.reason.c_str());}
          else {controls_.message("Applied. Values are live; dump parameters to save."); syncParameters();}
        }
      }
    } catch (const cv::Exception & e) {
      gui_failed_=true; RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"Preview: %s",e.what());
    } catch (const std::exception & e) {RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"Preview geometry: %s",e.what());}
  }
public:
  Node():rclcpp::Node("roi_lidar")
  {
#define DECLARE(name,field,getter) declare_parameter(name,config_.field); names_.emplace_back(name);
    ROI_PARAMETERS(DECLARE)
#undef DECLARE
    rcl_interfaces::msg::ParameterDescriptor ro; ro.read_only=true;
    device_id_=declare_parameter<std::string>("device_id","",ro);
    frame_id_=declare_parameter<std::string>("frame_id","front_axle_bev",ro);
    camera_x_=declare_parameter<double>("bev.camera_x_m",-.16,ro);
    camera_y_=declare_parameter<double>("bev.camera_y_m",0,ro);
    camera_yaw_=declare_parameter<double>("bev.camera_yaw_deg",0,ro);
    if (frame_id_.empty() || !std::isfinite(camera_x_) || !std::isfinite(camera_y_) || !std::isfinite(camera_yaw_)) {
      throw std::invalid_argument("Invalid frame or camera mount");
    }
    startup_=point_cloud::startupParameters(*this); startup_.device_id=device_id_;
    // oak_startup produces the same ROI image as BEV; render it on the main thread.
    const bool show_startup_roi=startup_.roi_preview_enabled;
    startup_.roi_preview_enabled=false;
    config_=readParameters();
#ifdef __linux__
    if (config_.gui && !std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
      gui_failed_=true;
      RCLCPP_WARN(get_logger(),"No desktop display; GUI disabled, image topics remain available");
    }
#endif
    startup_gui_requested_=show_startup_roi && config_.gui && !gui_failed_;
    if(startup_gui_requested_) {
      startup_.preview_callback=[this,last=Clock::time_point{}](const cv::Mat& image) mutable {
        const auto clock=Clock::now();
        if(clock-last<100ms)return;
        last=clock;auto copy=image.clone();
        std::lock_guard<std::mutex> lock(mutex_);startup_preview_=std::move(copy);
      };
    }
    RCLCPP_INFO(get_logger(),"Startup ground ROI: %dx%d, vertical offset=%d px, preview=%s; same oak_startup measurement as BEV",
      startup_.roi_width,startup_.roi_height,startup_.roi_vertical_offset_px,startup_gui_requested_?"on":"off");
    last_parameters_=get_parameters(names_);
    const auto qos=rclcpp::SensorDataQoS().keep_last(1);
    scan_pub_=create_publisher<sensor_msgs::msg::LaserScan>("~/scan",qos);
    obstacles_pub_=create_publisher<geometry_msgs::msg::PoseArray>("~/obstacles",qos);
    terrain_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("~/terrain",qos);
    imu_pub_=create_publisher<sensor_msgs::msg::Imu>("~/imu",rclcpp::SensorDataQoS().keep_last(200));
    planning_terrain_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("~/terrain_for_planning",qos);
    raw_terrain_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("~/terrain_raw",qos);
    points_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("~/obstacle_points",qos);
    preview_pub_=create_publisher<sensor_msgs::msg::Image>("~/bev_preview",qos);
    lane_bev_pub_=create_publisher<sensor_msgs::msg::Image>("~/lane_bev",qos);
    roi_pub_=create_publisher<sensor_msgs::msg::Image>("~/roi_preview",qos);
    timing_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("~/processing_timing",qos);
    status_pub_=create_publisher<std_msgs::msg::String>("~/status",rclcpp::QoS(1).transient_local());
    const auto latch=rclcpp::QoS(1).reliable().transient_local();
    locked_pub_=create_publisher<std_msgs::msg::Bool>("~/parameters_locked",latch);
    running_sub_=create_subscription<std_msgs::msg::Bool>("/roi_vfh_test/running",latch,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if(msg->data)syncParameters();
        parameters_locked_=msg->data;
        bool ready=false;
        {std::lock_guard<std::mutex> lock(mutex_);ready=snapshot_.revision==revision_ && snapshot_.state=="STREAMING" &&
          freshCapture(snapshot_.capture,seconds(Clock::now()),config_.max_age);}
        std_msgs::msg::Bool ack; ack.data=msg->data && ready; locked_pub_->publish(ack);
      });
    callback_=add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
      rcl_interfaces::msg::SetParametersResult out;
      if (parameters_locked_) {out.reason="VFH preview running: press Stop before changing ROI parameters"; return out;}
      try {auto c=readParameters(); for (const auto & p:ps) {apply(c,p);} validate(c); out.successful=true;}
      catch (const std::exception & e) {out.reason=e.what();} return out;
    });
    sync_timer_=create_wall_timer(50ms,[this](){syncParameters();});
    preview_timer_=create_wall_timer(5ms,[this](){preview();});
    worker_=std::thread([this](){run();});
  }
  ~Node() override
  {
    stop_=true; if (worker_.joinable()) {worker_.join();}
    if (windows_ || startup_window_) {try {cv::destroyAllWindows();} catch (...) {}}
  }
};
}
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<roi_lidar::Node>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("roi_lidar"),"%s",e.what()); rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}
