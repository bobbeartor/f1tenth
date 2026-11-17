"""Compile real drive readiness and IR capture gating code without ROS/GPU."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from test_preview_cpp import function

SOURCE = Path(__file__).resolve().parents[3]


class NativeGateTests(unittest.TestCase):
    def test_model_wait_blocks_control_and_sensor_jitter_does_not_drop_ir(self):
        compiler = shutil.which('c++')
        if not compiler:
            self.skipTest('C++ compiler unavailable')
        source = (SOURCE/'auto_control/src/auto_control_node.cpp').read_text()
        stop_reason = function(source, '  std::optional<std::string> stop_reason(')
        code = r'''
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include "auto_control/model_startup_gate.hpp"
#include "auto_control/start_authorization.hpp"
#include "camera_driver/tunnel_ir_input_policy.hpp"
#include "ir_camera_driver/ir_frame_rate_gate.hpp"
using namespace std::chrono_literals;
using auto_control::ModelStartupGate;
double seconds(std::int64_t ns) {return ns/1e9;}
void require(bool value) {if (!value) {throw std::runtime_error("readiness/IR regression");}}
struct Controller {
  bool enabled_{true},model_startup_check_required_{true},vesc_connected_{true};
  ModelStartupGate model_startup_gate_;
  bool startup_safety_enabled_{false},startup_manual_{false},camera_startup_ready_{false};
  bool operator_stopped_{false};
  auto_control::StartAuthorization start_authorization_;
  std::string control_mode_{"monitor_only"},avoidance_status_;
  bool avoidance_control_enabled_{false},avoidance_deformation_only_{false};
  bool avoidance_path_{false},avoidance_follow_centerline_{false};
  std::optional<std::int64_t> last_erpm_ns_,avoidance_lane_ns_,avoidance_depth_ns_;
  std::chrono::steady_clock::time_point avoidance_received_at_{};
  double erpm_timeout_sec_{.15},avoidance_max_age_{.2};
  int avoidance_confirmations_{0};
  bool path_{true},path_hold_active_{false};
  std::optional<std::int64_t> last_path_received_ns_{1000000000},last_valid_path_received_ns_{1000000000},path_capture_ns_{1000000000};
  double path_hold_timeout_sec_{.08},path_timeout_sec_{.15},path_capture_maximum_age_sec_{.2};
STOP_REASON
};
int main() {
  ModelStartupGate gate;
  const auto start=ModelStartupGate::Clock::now();
  require(!gate.ready(start));
  gate.observe(false,start); require(!gate.ready(start));
  gate.observe(true,start); require(gate.ready(start));
  require(!gate.ready(start+3001ms));
  require(!gate.ready(start-1ms));
  gate.observe(true,start+4s); require(gate.ready(start+4s));
  gate.observe(false,start+4s); require(!gate.ready(start+4s));
  Controller controller;
  require(controller.stop_reason(1000000000)=="waiting_for_models");
  controller.model_startup_gate_.observe(true,ModelStartupGate::Clock::now());
  require(!controller.stop_reason(1000000000));
  controller.model_startup_gate_.observe(false,ModelStartupGate::Clock::now());
  require(controller.stop_reason(1000000000)=="waiting_for_models");
  controller.model_startup_check_required_=false;
  require(!controller.stop_reason(1000000000));
  using camera_driver::TunnelIrRejection;
  using camera_driver::tunnelIrRejection;
  // Standalone-like validation consumes a new capture even during exposure
  // transition or with old camera timestamps; guarded mode still rejects it.
  require(tunnelIrRejection(true,true,start,start-1s,start+1s,start,5000,400)==TunnelIrRejection::none);
  require(tunnelIrRejection(false,true,start,start-1s,start+1s,start,5000,400)==TunnelIrRejection::time);
  require(tunnelIrRejection(true,true,start+100ms,start,start+100ms,start,5000,400)==TunnelIrRejection::none);
  require(tunnelIrRejection(false,true,start+100ms,start,start+100ms,start,30000,800)==TunnelIrRejection::time);
  require(tunnelIrRejection(false,true,start+200ms,start,start+210ms,start,5000,400)==TunnelIrRejection::exposure);
  require(tunnelIrRejection(false,true,start+200ms,start,start+210ms,start,30000,800)==TunnelIrRejection::none);
  require(tunnelIrRejection(true,true,start,start,start+1s,start,30000,800)==TunnelIrRejection::time);
  ir_camera_driver::IrFrameRateGate ir;
  int count=0;
  for (int i=0;i<300;++i) {
    const double capture=10.+i/30.+(i%2 ? .006 : -.006);
    count+=ir.accept(capture,30.);
    require(!ir.accept(capture,30.));
  }
  require(count==300);
  ir_camera_driver::IrFrameRateGate standby;
  count=0;
  for (int i=0;i<300;++i) {count+=standby.accept(i/30.,5.);}
  require(count==50);
  require(!standby.accept(11.,0.));
  require(standby.accept(12.,30.));
}
'''.replace('STOP_REASON', stop_reason)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            file, binary = root/'gates.cpp', root/'gates'
            file.write_text(code)
            result = subprocess.run([compiler, '-std=c++17', '-O2', '-DNDEBUG', '-Wall', '-Wextra',
                                     '-Werror', '-I'+str(SOURCE/'auto_control/include'),
                                     '-I'+str(SOURCE/'ir_camera_driver/include'),
                                     '-I'+str(SOURCE/'camera_driver/include'), str(file), '-o', str(binary)],
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
