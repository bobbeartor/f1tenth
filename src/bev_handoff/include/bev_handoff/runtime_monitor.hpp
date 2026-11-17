#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <sstream>
#include <vector>

namespace bev_handoff {
class RuntimeMonitor {
  using Clock=std::chrono::steady_clock;
  struct Samples {Clock::time_point start;std::vector<double> ms;double age_sum{0};};
  std::atomic<bool> enabled_{false};
  std::atomic<std::int64_t> deadline_ns_{0};
  std::mutex mutex_;
  std::map<std::string,Samples> samples_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  std::string node_;
public:
  explicit RuntimeMonitor(rclcpp::Node& node):node_(node.get_fully_qualified_name()) {
    publisher_=node.create_publisher<std_msgs::msg::String>("/auto/runtime_metrics",rclcpp::QoS(20));
    // Humble only supports volatile durability on its intra-process path.
    // Keep the GUI enable state latched for components loaded afterwards.
    rclcpp::SubscriptionOptions options;
    options.use_intra_process_comm=rclcpp::IntraProcessSetting::Disable;
    subscription_=node.create_subscription<std_msgs::msg::Bool>("/auto/runtime_metrics/enabled",
      rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::Bool::ConstSharedPtr p) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!enabled() || !p->data)samples_.clear();
        deadline_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
          (Clock::now()+std::chrono::seconds(2)).time_since_epoch()).count());
        enabled_.store(p->data);
      },options);
  }
  bool enabled()const{return enabled_.load(std::memory_order_relaxed) &&
    std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count()<deadline_ns_.load();}
  void sample(const std::string& stage,double ms,double age_ms=0) {
    if(!enabled() || !std::isfinite(ms) || ms<0)return;
    const auto now=Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if(!enabled())return;
    auto& s=samples_[stage];
    if(s.ms.empty())s.start=now;
    s.ms.push_back(ms);s.age_sum+=std::max(0.,age_ms);
    const double seconds=std::chrono::duration<double>(now-s.start).count();
    if(seconds<1. && s.ms.size()<4096)return;
    double total=0;for(double v:s.ms)total+=v;
    const double count=s.ms.size(),mean=total/count;
    std::sort(s.ms.begin(),s.ms.end());
    std::ostringstream out;
    out<<"{\"node\":\""<<node_<<"\",\"stage\":\""<<stage
      <<"\",\"fps\":"<<(count-1)/std::max(.001,seconds)
      <<",\"mean_ms\":"<<mean<<",\"p95_ms\":"<<s.ms[std::min(s.ms.size()-1,std::size_t(std::ceil(count*.95)-1))]
      <<",\"age_ms\":"<<s.age_sum/count<<"}";
    std_msgs::msg::String message;message.data=out.str();publisher_->publish(message);s={};
  }
};
class RuntimeSample {
  RuntimeMonitor* monitor_;
  const char* stage_;
  std::chrono::steady_clock::time_point start_;
public:
  RuntimeSample(RuntimeMonitor* monitor,const char* stage):monitor_(monitor && monitor->enabled()?monitor:nullptr),stage_(stage) {
    if(monitor_)start_=std::chrono::steady_clock::now();
  }
  ~RuntimeSample() {
    if(monitor_)try {
      monitor_->sample(stage_,std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-start_).count());
    }catch(const std::exception&) {}
  }
};
}
