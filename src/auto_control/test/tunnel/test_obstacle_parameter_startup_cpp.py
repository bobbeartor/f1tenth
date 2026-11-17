"""Run the actual planner callback against delayed detector declarations."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from test_preview_cpp import function

SOURCE = Path(__file__).resolve().parents[3]


class ObstacleParameterStartupTests(unittest.TestCase):
    def test_delayed_declarations_and_read_only_and_live_parameter_validation(self):
        compiler = shutil.which('c++')
        if not compiler:
            self.skipTest('C++ compiler unavailable')
        worker = (SOURCE / 'auto_control/include/auto_control/avoidance_worker.hpp').read_text()
        planner = (SOURCE / 'auto_control/include/auto_control/avoidance_planner.hpp').read_text()
        detector = (SOURCE / 'auto_control/src/obstacle_worker.cpp').read_text()
        scan = (SOURCE / 'auto_control/include/auto_control/obstacle_scan.hpp').read_text()
        clusters = (SOURCE / 'auto_control/include/auto_control/obstacle_clusters.hpp').read_text()
        declarations = detector[detector.index('  parameter(node,"roi.x"'):detector.index('  obstacle::validate(scan_)')]
        code = r'''
#include <cassert>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
namespace rcl_interfaces::msg {
struct ParameterDescriptor {bool read_only=false;std::string description;};
struct SetParametersResult {bool successful=false;std::string reason;};
}
namespace rclcpp {
enum class ParameterType {PARAMETER_DOUBLE, PARAMETER_INTEGER, PARAMETER_BOOL};
struct Parameter {
  std::string name; double value; ParameterType type;
  Parameter(std::string n,double v):name(n),value(v),type(ParameterType::PARAMETER_DOUBLE) {}
  Parameter(std::string n,int v):name(n),value(v),type(ParameterType::PARAMETER_INTEGER) {}
  Parameter(std::string n,bool v):name(n),value(v),type(ParameterType::PARAMETER_BOOL) {}
  const std::string & get_name()const {return name;}
  ParameterType get_type()const {return type;}
  double as_double()const {return value;}
};
struct Node {
  std::map<std::string,bool> readonly;
  std::function<rcl_interfaces::msg::SetParametersResult(const std::vector<Parameter>&)> callback;
  template<class T> T declare_parameter(const std::string& name,T value,
                                        rcl_interfaces::msg::ParameterDescriptor descriptor) {
    const auto result=callback({Parameter(name,value)});
    if(!result.successful) throw std::runtime_error(name+": "+result.reason);
    readonly[name]=descriptor.read_only;return value;
  }
  bool set(const std::vector<Parameter>& values) {
    // rclcpp enforces declared read-only descriptors before invoking callbacks.
    for(const auto & v:values)if(readonly[v.name])return false;
    return callback(values).successful;
  }
};
}
namespace auto_control::avoidance {
OPTIONS;
VALIDATE
}
namespace auto_control::obstacle {
SCAN;
CLUSTERS;
}
using namespace auto_control;
PARAMETER_HELPER
struct PlannerHarness {
  std::mutex mutex_;
  std::condition_variable wake_;
  avoidance::Options options_;
  double configured_fps_=80,extra_margin_=0;
  bool configuration_dirty_=false;
  LIVE_OPTIONS
  explicit PlannerHarness(rclcpp::Node & node) {node.callback=CALLBACK;}
};
struct DetectorHarness {
  obstacle::Options scan_;
  obstacle::ClusterOptions cluster_;
  double max_age_=.25,max_sync_=.04;
  explicit DetectorHarness(rclcpp::Node & node) {
DECLARATIONS
  }
};
int main() {
  rclcpp::Node node;
  PlannerHarness planner(node); // Existing callback is active BEFORE calibration completes.
  DetectorHarness detector(node); // Real delayed startup parameter declarations must succeed.
  assert(node.readonly.at("obstacles.roi.x"));
  assert(node.readonly.at("obstacles.scan.bins"));
  assert(!planner.configuration_dirty_);
  assert(!node.set({{"obstacles.roi.x",.2}})); // Startup-only remains protected.
  assert(node.set({{"obstacles.avoidance.max_fps",40.}}));
  assert(planner.configured_fps_==40.);
  assert(planner.configuration_dirty_);
  planner.configuration_dirty_=false;
  assert(!node.set({{"obstacles.avoidance.max_fps",81.}}));
  assert(planner.configured_fps_==40.);
  assert(!planner.configuration_dirty_);
  assert(!node.set({{"obstacles.avoidance.path_minimum_span_m",.3}}));
  assert(!node.set({{"obstacles.avoidance.max_fps",30.},{"obstacles.roi.x",.1}}));
  assert(planner.configured_fps_==40.);
  assert(!node.set({{"obstacles.avoidance.max_fps",30}})); // Wrong type rejected.
}
'''
        replacements = {
            'OPTIONS': function(planner, 'struct Options'),
            'VALIDATE': function(planner, 'inline void validate('),
            'SCAN': function(scan, 'struct Options'),
            'CLUSTERS': function(clusters, 'struct ClusterOptions'),
            'PARAMETER_HELPER': function(detector, 'template<class T> void parameter('),
            'LIVE_OPTIONS': function(worker, '  static const std::map<std::string,double avoidance::Options::*>& live_options()'),
            'CALLBACK': function(worker, '[this](const std::vector<rclcpp::Parameter>& values)'),
            'DECLARATIONS': declarations,
        }
        # Long names first: OPTIONS is a substring of LIVE_OPTIONS.
        for key in sorted(replacements, key=len, reverse=True):
            code = code.replace(key, replacements[key])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'check.cpp'
            path.write_text(code)
            executable = Path(directory) / 'check'
            subprocess.run([compiler, '-std=c++17', '-pthread', '-Wall', '-Wextra', '-Werror',
                            str(path), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)
