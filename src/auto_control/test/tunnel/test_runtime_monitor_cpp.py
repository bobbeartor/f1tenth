"""Exercise actual subscription construction against Humble's QoS constraint."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[3]


class RuntimeSubscriptionTests(unittest.TestCase):
    def test_latched_control_subscriptions_with_intra_process_node(self):
        compiler = shutil.which('c++')
        if not compiler:
            self.skipTest('C++ compiler unavailable')
        ir = (SOURCE / 'ir_camera_driver/src/shared_ir_bev_node.cpp').read_text()
        start = ir.index('    rclcpp::SubscriptionOptions mode_options;')
        end = ir.index('    diagnostic_started_', start)
        # Execute the real subscription creation and callback, not a copied implementation.
        ir_subscription = ir[start:end]
        stub = r'''
#pragma once
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
namespace std_msgs { namespace msg {
struct Bool {using ConstSharedPtr=std::shared_ptr<const Bool>; bool data;};
struct String {using ConstSharedPtr=std::shared_ptr<const String>; std::string data;};
}}
namespace rclcpp {
enum class IntraProcessSetting {NodeDefault, Disable};
struct SubscriptionOptions {IntraProcessSetting use_intra_process_comm=IntraProcessSetting::NodeDefault;};
struct QoS {
  bool latched=false;
  explicit QoS(int) {}
  QoS& reliable() {return *this;}
  QoS& transient_local() {latched=true;return *this;}
};
template<class T> struct Publisher {
  using SharedPtr=std::shared_ptr<Publisher<T>>;
  void publish(const T&) {}
};
template<class T> struct Subscription {using SharedPtr=std::shared_ptr<Subscription<T>>;};
struct Node {
  std::function<void(std_msgs::msg::Bool::ConstSharedPtr)> enable;
  std::function<void(std_msgs::msg::String::ConstSharedPtr)> mode;
  const char* get_fully_qualified_name() {return "/test";}
  template<class T> typename Publisher<T>::SharedPtr create_publisher(const char*, QoS) {
    return std::make_shared<Publisher<T>>();
  }
  template<class T, class F> typename Subscription<T>::SharedPtr create_subscription(
      const char*, QoS qos, F callback, SubscriptionOptions options={}) {
    // Model the exact Humble failure observed on the Jetson with node-level IPC enabled.
    if (qos.latched && options.use_intra_process_comm!=IntraProcessSetting::Disable)
      throw std::runtime_error("intraprocess communication allowed only with volatile durability");
    if (!qos.latched) throw std::runtime_error("latched state was lost");
    if constexpr(std::is_same_v<T,std_msgs::msg::Bool>) enable=callback;
    else mode=callback;
    return std::make_shared<Subscription<T>>();
  }
};
}
'''
        code = r'''
#include <bev_handoff/runtime_monitor.hpp>
#include <cassert>
struct IrHarness : rclcpp::Node {
  using Clock=std::chrono::steady_clock;
  bool inside_=false;
  Clock::time_point retry_at_=Clock::now();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  IrHarness() {
IR_SUBSCRIPTION
  }
};
int main() {
  rclcpp::Node node;
  bev_handoff::RuntimeMonitor monitor(node);
  assert(!monitor.enabled());
  node.enable(std::make_shared<std_msgs::msg::Bool>(std_msgs::msg::Bool{true}));
  assert(monitor.enabled());
  node.enable(std::make_shared<std_msgs::msg::Bool>(std_msgs::msg::Bool{false}));
  assert(!monitor.enabled());
  IrHarness ir;
  ir.mode(std::make_shared<std_msgs::msg::String>(std_msgs::msg::String{"IR"}));
  assert(ir.inside_);
  ir.mode(std::make_shared<std_msgs::msg::String>(std_msgs::msg::String{"RGB"}));
  assert(!ir.inside_);
}
'''.replace('IR_SUBSCRIPTION', ir_subscription)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, text in {
                'rclcpp/rclcpp.hpp': stub,
                'std_msgs/msg/bool.hpp': '#pragma once\n',
                'std_msgs/msg/string.hpp': '#pragma once\n',
                'check.cpp': code,
            }.items():
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text)
            executable = root / 'check'
            subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            '-I' + str(root), '-I' + str(SOURCE / 'bev_handoff/include'),
                            str(root / 'check.cpp'), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    unittest.main()
