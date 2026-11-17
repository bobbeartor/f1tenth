#include "multiclass_yolo/yolox_detector.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
int main(int argc, char ** argv) {
 if(argc!=2) return 2;
 cv::Mat bgr(800,1280,CV_8UC3); cv::randu(bgr,0,255);
 auto net=cv::dnn::readNetFromONNX(argv[1]);
 cv::Mat resized; cv::resize(bgr,resized,{512,320});
 net.setInput(cv::dnn::blobFromImage(resized,1.0,{512,320},cv::Scalar(),false,false));
 auto raw=net.forward();
 if(raw.dims!=3 || raw.size[0]!=1 || raw.size[1]!=3360 || raw.size[2]!=9 || !cv::checkRange(raw))
   throw std::runtime_error("Expected finite [1,3360,9]");
 multiclass_yolo::YoloxDetector d(argv[1],"CPU","","fp32",512,320,0.0f,.65f,1024*1024);
 auto a=d.detect(bgr);
 cv::Mat rgb; cv::cvtColor(bgr,rgb,cv::COLOR_BGR2RGB);
 auto b=d.detect(rgb,true);
 if(a.detections.empty()) throw std::runtime_error("Expected nonzero low-score candidates for parity test");
 if(a.detections.size()!=b.detections.size()) throw std::runtime_error("RGB/BGR paths differ");
 for(std::size_t i=0;i<a.detections.size();++i) {
   if(a.detections[i].class_id!=b.detections[i].class_id ||
      std::abs(a.detections[i].score-b.detections[i].score)>1e-6 ||
      cv::norm(a.detections[i].box.tl()-b.detections[i].box.tl())>1e-5)
     throw std::runtime_error("RGB/BGR parity failed");
 }

 // Compare full-frame ROI detection with an explicitly cropped model input.
 const cv::Rect roi(0, 280, 1280, 520);
 const auto local = d.detect(bgr(roi));
 const auto mapped = d.detect(rgb, roi, true);
 if (local.detections.empty() || local.detections.size() != mapped.detections.size())
   throw std::runtime_error("Expected matching detections for the cropped input");
 for (std::size_t i = 0; i < local.detections.size(); ++i) {
   const auto & expected = local.detections[i];
   const auto & actual = mapped.detections[i];
   const cv::Point2f origin(roi.x, roi.y);
   if (expected.class_id != actual.class_id || std::abs(expected.score - actual.score) > 1e-6 ||
       cv::norm(expected.box.tl() + origin - actual.box.tl()) > 1e-4 ||
       cv::norm(expected.box.br() + origin - actual.box.br()) > 1e-4 ||
       actual.box.y < roi.y || actual.box.br().y > bgr.rows + 1e-4)
     throw std::runtime_error("ROI boxes must map to the original image and stay inside the ROI");
 }
 std::cout << "ONNX finite [1,3360,9]; CPU inference, RGB/BGR parity, and ROI mapping passed; boxes=" << a.detections.size() << '\n';
}
