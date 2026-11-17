#pragma once
#include "roi_vfh_test/planner.hpp"
#include <opencv2/imgproc.hpp>
#include <array>
#include <iomanip>
#include <sstream>

namespace roi_vfh_test {
struct PlannerField {
  const char* name;
  double Config::* member;
  double minimum,maximum;
  const char* unit;
  const char* label{nullptr};
  double scale{1}; // ROS/UI degrees, internal radians for angular fields.
};
inline constexpr std::size_t planner_field_count=46;
inline const std::array<PlannerField,planner_field_count>& plannerFields() {
  static const std::array<PlannerField,planner_field_count> fields{{
    {"nominal_speed_mps",&Config::speed,.05,3,"max speed, m/s; YAML ceiling"},
    {"lane_lookahead_m",&Config::lane_lookahead,.01,.8,"Stanley tangent, m (0.01..0.8)"},
    {"safety_margin_m",&Config::margin,0,.2,"each side / front, m (0..0.2)"},
    {"horizon_m",&Config::horizon,.3,2.5,"planning length, m (0.3..2.5)"},
    {"reference_distance_m",&Config::reference_distance,.3,3,"VFH target, m (0.3..3)"},
    {"sample_step_m",&Config::step,.02,.08,"path sample, m (0.02..0.08)"},
    {"deceleration_mps2",&Config::deceleration,.1,3,"m/s^2; startup ceiling applies"},
    {"response_delay_sec",&Config::delay,0,1,"response delay, sec (0..1)"},
    {"histogram_low",&Config::hist_low,0,999,"VFH low (must be < high)"},
    {"histogram_high",&Config::hist_high,.001,1000,"VFH high (must be > low)"},
    {"lane_memory_enabled",&Config::lane_memory_enabled,0,1,"last lane direction: 0=off / 1=on"},
    {"hazard_filter_enabled",&Config::hazard_filter_enabled,0,1,"0=off / 1=on"},
    {"hazard_confirm_frames",&Config::hazard_confirm_frames,1,5,"consecutive depth frames (integer)"},
    {"hazard_small_max_cells",&Config::hazard_small_max_cells,1,50,"small 8-connected component (integer)"},
    {"hazard_match_radius_cells",&Config::hazard_match_radius_cells,0,2,"motion-aligned search cells (integer)"},
    {"hazard_max_gap_sec",&Config::hazard_max_gap_sec,.02,.5,"max gap between observations, sec"},
    {"avoidance_curve_enabled",&Config::avoidance_curve_enabled,0,1,"extra checked curves: 0=off / 1=on"},
    {"avoidance_curve_samples",&Config::avoidance_curve_samples,3,17,"steering seeds; ODD integer 3..17"},
    {"avoidance_length_slack_m",&Config::avoidance_length_slack_m,0,.5,"length gap for comparing return costs, m"},
    {"avoidance_lane_weight",&Config::avoidance_lane_weight,0,5,"soft lane return weight (0..5)"},
    {"avoidance_steering_weight",&Config::avoidance_steering_weight,0,5,"steering continuity weight (0..5)"},
    {"avoidance_clearance_weight",&Config::clearance_weight,0,5,"forward-grid clearance preference (0..5)"},
    {"obstacle_uncertainty_m",&Config::uncertainty,0,.5,"position uncertainty, m (0..0.5)"},
    {"hazard_hold_sec",&Config::hazard_hold_sec,0,1,"confirmed flicker hold, sec (0..1)"},
    {"obstacle_memory_enabled",&Config::obstacle_memory_enabled,0,1,"side/rear memory: 0=off / 1=on"},
    {"obstacle_memory_sec",&Config::obstacle_memory_sec,0,5,"max time since confirmed, sec (0..5)"},
    {"obstacle_memory_distance_m",&Config::obstacle_memory_distance_m,.1,2,"retention distance from body, m"},
    {"obstacle_memory_clearance_m",&Config::obstacle_memory_clearance_m,0,.5,"extra travel past rear end, m"},
    {"target_recovery_acceleration_mps2",&Config::target_recovery_acceleration,.05,3,"target speed recovery, m/s^2","target_acceleration_mps2"},
    {"drive.longitudinal_start_duty",&Config::drive_start_duty,0,1,"Duty; must not exceed maximum_duty","start_duty"},
    {"drive.duty_rise_rate_per_sec",&Config::drive_rise,.001,1,"Duty/sec (not m/s^2)","duty_rise_rate_per_sec"},
    {"drive.duty_fall_rate_per_sec",&Config::drive_fall,.001,1,"Duty/sec; propulsion release","duty_fall_rate_per_sec"},
    {"drive.longitudinal_recovery_duty_rise_per_sec",&Config::drive_recovery_rise,.001,1,"Duty/sec after braking","recovery_duty_rise_per_sec"},
    {"drive.longitudinal_recovery_start_ramp_sec",&Config::drive_recovery_ramp,.01,5,"low-speed recovery ramp, sec","recovery_start_ramp_sec"},
    {"drive.brake_maximum_current_amps",&Config::drive_brake_max,.1,8,"A; normal + Stop brake limit","brake_maximum_current_amps"},
    {"drive.brake_current_rise_amps_per_sec",&Config::drive_brake_rise,.1,64,"A/sec; normal brake rise","brake_current_rise_A_per_sec"},
    {"drive.brake_current_fall_amps_per_sec",&Config::drive_brake_fall,.1,64,"A/sec; normal brake release","brake_current_fall_A_per_sec"},
    {"drive.longitudinal_brake_speed_gain",&Config::drive_brake_speed_gain,.01,10,"brake gain per excess speed","brake_speed_gain"},
    {"max_steering_deg",&Config::max_steer,5,60,"degrees; vehicle profile ceiling",nullptr,180/pi},
    {"steering_rate_deg_per_sec",&Config::steer_rate,10,1000,"degrees/sec; profile ceiling",nullptr,180/pi},
    {"steering_current_weight",&Config::steering_weight,.01,1,"0.01..1; larger = faster response"},
    {"stanley_gain",&Config::stanley_gain,0,10,"lateral error correction gain"},
    {"stanley_softening_speed_mps",&Config::stanley_softening,.01,3,"m/s; larger = softer low-speed steer"},
    {"max_curvature_per_m",&Config::max_curvature,.1,10,"1/m; also limited by max steer"},
    {"stanley_corner_heading_threshold_deg",&Config::stanley_corner_heading,0,180,"degrees; corner heading threshold","corner_heading_threshold_deg",180/pi},
    {"stanley_corner_opposing_correction_ratio",&Config::stanley_opposing,0,.99,"0..0.99; opposing correction ratio","corner_opposing_ratio"}
  }};
  return fields;
}
inline const PlannerField* plannerField(const std::string& name) {
  for(const auto& field:plannerFields())if(name==field.name)return &field;
  return nullptr;
}
inline void validatePlannerEdits(const Config& config,double deceleration_ceiling,double speed_ceiling=3,
    double steering_ceiling=pi/3,double steering_rate_ceiling=20,double maximum_duty=1) {
  for(const auto& f:plannerFields()) {
    const double value=(config.*(f.member))*f.scale;
    const double maximum=f.member==&Config::drive_start_duty ? maximum_duty : f.maximum;
    if(!std::isfinite(value) || value<f.minimum || value>maximum) {
      std::ostringstream error;
      error<<(f.label?f.label:f.name)<<": enter "<<f.minimum<<".."<<maximum;
      throw std::invalid_argument(error.str());
    }
  }
  if(config.hist_low>=config.hist_high)throw std::invalid_argument("histogram_low must be smaller than histogram_high");
  if(config.deceleration>deceleration_ceiling+1e-9)
    throw std::invalid_argument("deceleration_mps2 exceeds startup controller ceiling");
  if(config.speed>speed_ceiling+1e-9)throw std::invalid_argument("nominal_speed_mps exceeds source controller maximum_speed_mps");
  if(config.max_steer>steering_ceiling+1e-9)throw std::invalid_argument("max_steering_deg exceeds startup vehicle steering ceiling");
  if(config.steer_rate>steering_rate_ceiling+1e-9)throw std::invalid_argument("steering_rate_deg_per_sec exceeds startup vehicle steering-rate ceiling");
  config.validate();
}
class PlannerControls {
  struct Edit {double observed{};std::string draft;bool pending{false};};
  std::array<Edit,planner_field_count> edits_;
  bool ready_{false},open_{false},select_all_{false};
  int active_{-1};
  std::size_t page_{0};
  static constexpr std::size_t page_size=12;
  static constexpr std::size_t pages=(planner_field_count+page_size-1)/page_size;
  std::string message_{"Edit values, then Apply. Start also applies valid edits."};
  static std::string format(double value){std::ostringstream s;s<<std::setprecision(8)<<value;return s.str();}
  static cv::Rect field(std::size_t i){i%=page_size;return {250+int(i/6)*374,166+int(i%6)*48,104,30};}
  static cv::Rect previousButton(){return {548,106,66,26};}
  static cv::Rect nextButton(){return {622,106,66,26};}
  void changePage(std::size_t page){page_=page;active_=-1;}
  static cv::Rect applyButton(){return {16,488,350,36};}
  static cv::Rect reloadButton(){return {390,488,350,36};}
public:
  enum class Action {None,Apply,Reload};
  static cv::Rect toggleButton(){return {366,12,170,36};}
  bool editing()const{return active_>=0;}
  void blur(){active_=-1;}
  void show(bool value){open_=value;active_=-1;}
  void toggle(){show(!open_);}
  void message(const std::string& text){message_=text;}
  void sync(const Config& config) {
    for(std::size_t i=0;i<edits_.size();++i) {
      auto& e=edits_[i];const auto& f=plannerFields()[i];const double value=(config.*(f.member))*f.scale;
      if(!ready_ || value!=e.observed){e.observed=value;e.draft=format(value);e.pending=false;}
    }
    ready_=true;
  }
  void reload(const Config& config){ready_=false;active_=-1;sync(config);}
  std::vector<std::pair<std::string,double>> changes()const {
    std::vector<std::pair<std::string,double>> result;
    for(std::size_t i=0;i<edits_.size();++i) {
      const auto& e=edits_[i];if(!e.pending)continue;
      std::size_t used=0;double value=0;
      try {value=std::stod(e.draft,&used);}catch(const std::exception&){throw std::invalid_argument(std::string(plannerFields()[i].name)+": enter a number");}
      if(used!=e.draft.size() || !std::isfinite(value))throw std::invalid_argument(std::string(plannerFields()[i].name)+": enter a finite number");
      if(value!=e.observed)result.emplace_back(plannerFields()[i].name,value);
    }
    return result;
  }
  Action click(int x,int y,bool locked) {
    if(!open_)return Action::None;
    if(previousButton().contains(cv::Point(x,y))){changePage((page_+pages-1)%pages);return Action::None;}
    if(nextButton().contains(cv::Point(x,y))){changePage((page_+1)%pages);return Action::None;}
    if(locked)return Action::None;
    for(std::size_t i=page_*page_size;i<std::min(edits_.size(),(page_+1)*page_size);++i)if(field(i).contains(cv::Point(x,y))){active_=int(i);select_all_=true;return Action::None;}
    active_=-1;
    if(applyButton().contains(cv::Point(x,y)))return Action::Apply;
    if(reloadButton().contains(cv::Point(x,y)))return Action::Reload;
    return Action::None;
  }
  Action key(int key,bool locked) {
    if(!open_){active_=-1;return Action::None;}
    if(key=='[' || key==']'){changePage((page_+(key==']'?1:pages-1))%pages);return Action::None;}
    if(locked){active_=-1;return Action::None;}
    if(key==13 || key==10 || key=='a' || key=='A'){active_=-1;return Action::Apply;}
    if(key==9){active_=active_<0?int(page_*page_size):(active_+1)%int(edits_.size());page_=active_/page_size;select_all_=true;return Action::None;}
    if(active_<0)return Action::None;
    auto& e=edits_[active_];
    if(key==27){e.draft=format(e.observed);e.pending=false;active_=-1;return Action::None;}
    if(key==8 || key==127 || key==0xff08) {
      if(select_all_)e.draft.clear();else if(!e.draft.empty())e.draft.pop_back();
    }else if((key>='0' && key<='9') || key=='.' || key=='-' || key=='+' || key=='e' || key=='E') {
      if(select_all_)e.draft.clear();if(e.draft.size()<24)e.draft.push_back(static_cast<char>(key));
    }else return Action::None;
    select_all_=false;e.pending=true;return Action::None;
  }
  void draw(cv::Mat& canvas,bool locked,double deceleration_ceiling,double speed_ceiling=3,
      double steering_ceiling=pi/3,double steering_rate_ceiling=20,double maximum_duty=1)const {
    const auto text=[&](const std::string& value,cv::Point at,double scale,cv::Scalar color){cv::putText(canvas,value,at,cv::FONT_HERSHEY_SIMPLEX,scale,color,1,cv::LINE_AA);};
    cv::rectangle(canvas,toggleButton(),{120,110,75},-1);
    text(open_?"Back to view":"VFH parameters",{378,36},.49,{255,255,255});
    if(!open_)return;
    cv::rectangle(canvas,{0,102,760,518},{245,245,245},-1);
    text(locked?"VFH PARAMETERS - LOCKED until Stop":"VFH PARAMETERS - editable before Start",{16,124},.47,{35,35,35});
    cv::rectangle(canvas,previousButton(),{180,180,180},-1);cv::rectangle(canvas,nextButton(),{180,180,180},-1);
    text("< Prev",{555,124},.37,{35,35,35});text("Next >",{628,124},.37,{35,35,35});
    text(std::to_string(page_+1)+" / "+std::to_string(pages),{697,124},.38,{35,35,35});
    text("Click/type. Enter/A: Apply all pages. Tab: next. [ / ]: page. Esc: cancel.",{16,146},.40,{60,60,60});
    for(std::size_t i=page_*page_size;i<std::min(edits_.size(),(page_+1)*page_size);++i) {
      const auto& e=edits_[i];const auto& f=plannerFields()[i];const auto box=field(i);const int left=16+int((i%page_size)/6)*374;
      text(f.label?f.label:f.name,{left,box.y+12},.37,{35,35,35});
      const std::string unit=f.member==&Config::drive_start_duty
        ? "Duty fraction, 0.." + format(maximum_duty) + " (0.053 = 5.3%)"
        : f.unit;
      text(unit,{left,box.y+30},.28,{100,100,100});
      cv::rectangle(canvas,box,locked?cv::Scalar(218,218,218):(e.pending?cv::Scalar(220,245,255):cv::Scalar(255,255,255)),-1);
      cv::rectangle(canvas,box,active_==int(i)?cv::Scalar(220,120,30):cv::Scalar(150,150,150),active_==int(i)?2:1);
      const auto value=e.draft.size()>11?e.draft.substr(e.draft.size()-11):e.draft;
      text(value+(active_==int(i)?"_":""),box.tl()+cv::Point(5,21),.44,{25,25,25});
    }
    cv::rectangle(canvas,applyButton(),locked?cv::Scalar(160,160,160):cv::Scalar(75,150,70),-1);
    cv::rectangle(canvas,reloadButton(),{175,175,175},-1);
    text("Apply edits (Enter / A)",{70,512},.48,{255,255,255});text("Reload / discard edits",{440,512},.48,{35,35,35});
    text(message_.substr(0,104),{16,550},.39,{35,65,190});
    text(cv::format("Ceilings: v %.2f | decel %.2f | steer %.1fdeg | rate %.0fdeg/s",speed_ceiling,deceleration_ceiling,
      steering_ceiling*180/pi,steering_rate_ceiling*180/pi),{16,576},.39,{60,60,60});
    text("Start applies edits and locks ROI + VFH. Stop unlocks both.",{16,601},.40,{60,60,60});
  }
};
}
