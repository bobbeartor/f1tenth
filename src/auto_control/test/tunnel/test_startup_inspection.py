import ast
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch
from auto_control_tunnel.inspection_state import InspectionState
from test_preview_cpp import function

SOURCE = Path(__file__).resolve().parents[3]


def load_ui():
    modules = {name: ModuleType(name) for name in (
        'rclpy', 'rclpy.node', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg',
        'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv')}
    modules['rclpy.node'].Node = object
    for name in ('QoSProfile', 'ReliabilityPolicy', 'DurabilityPolicy'):
        setattr(modules['rclpy.qos'], name, object)
    modules['sensor_msgs.msg'].Image = NS
    modules['std_msgs.msg'].String = NS
    modules['std_srvs.srv'].Trigger = NS(Request=lambda: object())
    spec = importlib.util.spec_from_file_location('auto_control_tunnel._inspection_test',
        SOURCE / 'auto_control/auto_control_tunnel/startup_inspection.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    return module


class InspectionTests(unittest.TestCase):
    def ready_state(self):
        state = InspectionState(['rgb','ir'])
        state.camera = 'READY | measured pose'
        state.models = dict.fromkeys(state.models, 'READY | trial inference=PASS')
        return state

    def test_missing_checks_and_stale_controller_never_enable_button(self):
        state = InspectionState(['rgb','ir'])
        state.update_controller('READY_TO_START | ready', 10)
        self.assertFalse(state.can_start(10))
        state.camera = 'READY | pose'
        state.models['rgb'] = 'READY | trial inference=PASS'
        self.assertFalse(state.can_start(10))
        state.models['ir'] = 'READY | trial inference=PASS'
        self.assertTrue(state.can_start(10.5))
        self.assertFalse(state.can_start(11.1))
        state.models['ir'] = 'FAILED | engine error'
        self.assertFalse(state.can_start(10.5))

    def test_inspection_closes_only_after_controller_ack_or_manual_checks(self):
        state = self.ready_state()
        state.update_controller('READY_TO_START | ready', 10)
        self.assertFalse(state.completed(10))
        state.update_controller('STARTED | automatic duty permitted',10)
        self.assertTrue(state.completed(10))
        self.assertFalse(state.completed(11.1))
        state.update_controller('MANUAL | skip operator button',10)
        self.assertTrue(state.completed(10))
        self.assertFalse(state.can_start(10))
        state.camera='MEASURING | restarted'
        self.assertFalse(state.completed(10))

    def test_click_is_not_an_enable_topic_and_requires_fresh_ready_state(self):
        module = load_ui()
        panel = module.StartupInspection.__new__(module.StartupInspection)
        panel.state=self.ready_state()
        panel.pending=None
        calls=[]
        callbacks=[]
        panel.client=NS(service_is_ready=lambda: True,
            call_async=lambda req: calls.append(req) or NS(add_done_callback=callbacks.append))
        panel.request_start()
        self.assertFalse(calls)
        panel.state.update_controller('READY_TO_START | ready')
        panel.request_start()
        self.assertEqual(len(calls),1)
        panel.request_start()
        self.assertEqual(len(calls),1)
        callbacks[0](NS(result=lambda: NS(success=False,message='centerline_stale')))
        self.assertEqual(panel.state.error,'centerline_stale')
        self.assertFalse(panel.state.completed())

    def test_render_shows_camera_models_and_button_without_ros(self):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest('Python OpenCV unavailable')
        module=load_ui()
        state=self.ready_state()
        state.update_controller('READY_TO_START | ready',10)
        image=np.full((400,640,3),110,np.uint8)
        ready=module.render(state,image,True,False,now=10)
        blocked=module.render(state,image,True,False,now=12)
        self.assertEqual(ready.shape,(867,1020,3))
        self.assertEqual(tuple(ready[round(665*.85),round(30*.85)]),(63,129,18))
        self.assertEqual(tuple(blocked[round(665*.85),round(30*.85)]),(168,176,170))
        self.assertTrue(np.any(ready[70:340,30:490]))

    def test_actual_controller_checks_and_final_output_gate(self):
        compiler=shutil.which('c++')
        if not compiler:
            self.skipTest('C++ compiler unavailable')
        source=(SOURCE/'auto_control/src/auto_control_node.cpp').read_text()
        stop=function(source,'  std::optional<std::string> stop_reason(')
        refresh=function(source,'  void refresh_start_authorization(')
        publish=function(source,'  void publish_commands(')
        # Test the actual final output guard, ending before unrelated diagnostics.
        guard=publish[publish.index('{')+1:publish.index('    if (motor_mode == "brake"')]
        code=r'''
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include "auto_control/model_startup_gate.hpp"
#include "auto_control/start_authorization.hpp"
double seconds(std::int64_t n) {return n/1e9;}
void require(bool x) {if(!x) throw std::runtime_error("startup safety failure");}
struct Controller {
  bool operator_stopped_{false};
  bool enabled_{true},model_startup_check_required_{true},vesc_connected_{true};
  bool startup_safety_enabled_{true},startup_manual_{false},camera_startup_ready_{false};
  auto_control::StartAuthorization start_authorization_;
  auto_control::ModelStartupGate model_startup_gate_;
  std::string control_mode_{"drive"},avoidance_status_;
  bool avoidance_control_enabled_{false},avoidance_deformation_only_{false};
  bool avoidance_path_{false},avoidance_follow_centerline_{false};
  std::optional<std::int64_t> last_erpm_ns_{1000000000},avoidance_lane_ns_,avoidance_depth_ns_;
  std::chrono::steady_clock::time_point avoidance_received_at_{};
  double erpm_timeout_sec_{.15},avoidance_max_age_{.2}; int avoidance_confirmations_{0};
  bool path_{true},path_hold_active_{false};
  std::optional<std::int64_t> last_path_received_ns_{1000000000},last_valid_path_received_ns_{1000000000},path_capture_ns_{1000000000};
  double path_hold_timeout_sec_{.08},path_timeout_sec_{.15},path_capture_maximum_age_sec_{.2};
  struct Now {std::int64_t nanoseconds() const {return 1000000000;}};
  Now now() const {return {};}
STOP
REFRESH
  double final_duty(double duty) {
    const std::string motor_mode="duty";
GUARD
    return duty;
  }
  bool request() {refresh_start_authorization(); const bool ok=start_authorization_.request(!stop_reason(1000000000,false)); if(ok)operator_stopped_=false; return ok;}
};
int main() {
  Controller c;
  require(c.stop_reason(1000000000)=="waiting_for_camera_pose");
  require(!c.request() && c.final_duty(.1)==0);
  c.camera_startup_ready_=true;
  require(c.stop_reason(1000000000)=="waiting_for_models");
  require(!c.request());
  c.model_startup_gate_.observe(true,std::chrono::steady_clock::now());
  require(c.stop_reason(1000000000)=="waiting_for_start_button");
  require(c.final_duty(.1)==0); // Ready alone never authorizes motion.
  c.path_=false; require(!c.request()); c.path_=true;
  require(c.final_duty(.1)==0 && c.request());
  require(!c.stop_reason(1000000000) && c.final_duty(.1)==.1);
  c.operator_stopped_=true;c.start_authorization_.revoke();
  require(c.stop_reason(1000000000)=="operator_stop" && c.final_duty(.1)==0);
  require(c.request() && c.final_duty(.1)==.1);
  c.vesc_connected_=false; require(c.final_duty(.1)==0);
  c.vesc_connected_=true; require(c.final_duty(.1)==0 && c.request());
  c.last_erpm_ns_=0; require(c.final_duty(.1)==0 && !c.request());
  c.last_erpm_ns_=1000000000; require(c.final_duty(.1)==0 && c.request());
  c.model_startup_gate_.observe(false,std::chrono::steady_clock::now());
  require(c.final_duty(.1)==0 && !c.request());
  c.model_startup_gate_.observe(true,std::chrono::steady_clock::now());
  require(c.final_duty(.1)==0 && c.request());
  c.model_startup_gate_.observe(true,std::chrono::steady_clock::now()-std::chrono::seconds(4));
  require(c.final_duty(.1)==0 && !c.request());
  c.model_startup_gate_.observe(true,std::chrono::steady_clock::now());
  require(c.final_duty(.1)==0 && c.request());
  c.startup_manual_=true; require(c.final_duty(.1)==0 && !c.request());
  c.startup_manual_=false; require(c.final_duty(.1)==0 && c.request());
  c.enabled_=false; require(c.final_duty(.1)==0 && !c.request());
  c.enabled_=true; require(c.final_duty(.1)==0 && c.request());
  c.camera_startup_ready_=false; require(c.final_duty(.1)==0);
  c.camera_startup_ready_=true; require(c.final_duty(.1)==0);
  c.startup_safety_enabled_=false; require(c.final_duty(.1)==.1); // Legacy standalone mode.
}
'''.replace('STOP',stop).replace('REFRESH',refresh).replace('GUARD',guard)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'startup.cpp'
            path.write_text(code)
            binary=Path(directory)/'startup'
            result=subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror',
                '-I'+str(SOURCE/'auto_control/include'),str(path),'-o',str(binary)],
                text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            subprocess.run([str(binary)],check=True)


if __name__=='__main__':
    unittest.main()
