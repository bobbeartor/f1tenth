#include "multiclass_yolo/postprocess.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
void check(bool v, const char * msg) {if (!v) throw std::runtime_error(msg);}
int main() {
 using multiclass_yolo::decode_detections;
 std::array<float,4> thresholds{{.25f,.25f,.25f,.25f}};
 float rows[][9] = {
  {100,100,40,40,.9f,.9f,.01f,.01f,.01f},
  {100,100,40,40,.8f,.9f,.01f,.01f,.01f},
  {100,100,40,40,.9f,.01f,.9f,.01f,.01f},
  {200,100,40,40,.5f,.01f,.01f,.8f,.01f},
  {300,100,40,40,.5f,.01f,.01f,.01f,.8f},
  {0,0,20,20,.9f,.9f,.01f,.01f,.01f},
  {100,100,-40,40,.9f,.9f,.01f,.01f,.01f},
  {100,100,40,40,std::numeric_limits<float>::quiet_NaN(),.9f,.01f,.01f,.01f},
  {100,100,40,40,.1f,.9f,.01f,.01f,.01f}
 };
 auto r=decode_detections(&rows[0][0],9,9,.5f,1024,640,0,0,thresholds,.65f);
 check(r.size()==5,"class-aware NMS and invalid/low-score rejection");
 check(r[0].class_id==0 && std::abs(r[0].score-.81f)<1e-5,"objectness times class score");
 check(r[0].box.x==160 && r[0].box.width==80,"inverse resize");
 bool clipped=false; for (auto & d:r) if (d.box.x==0 && d.box.y==0) clipped=d.box.width==20;
 check(clipped,"clip at image border");
 thresholds[2]=.41f;
 r=decode_detections(&rows[0][0],9,9,1,512,320,10,20,thresholds,.65f);
 check(r.size()==4,"independent sedan threshold");
 for (auto & d:r) check(d.class_id!=2 && d.box.x>=10 && d.box.y>=20,"class filtering and origin");
 bool threw=false; try {decode_detections(&rows[0][0],9,6,1,512,320,0,0,thresholds,.65f);} catch(const std::invalid_argument &) {threw=true;}
 check(threw,"reject incorrect output shape");
 std::cout << "Postprocess tests passed\n";
}
