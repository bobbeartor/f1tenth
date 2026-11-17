import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import unittest

from auto_control_tunnel.inspection_state import InspectionState
from test_preview_cpp import function

SOURCE = Path(__file__).resolve().parents[3]


class LoadingProgressTests(unittest.TestCase):
    def test_heartbeat_does_not_reset_stage_timer_or_authorize_start(self):
        state = InspectionState(['traffic'])
        state.started = 0.
        state.camera = 'READY | pose'
        state.update_controller('READY_TO_START | ready', 80.)
        state.update_model('traffic', 'LOADING | Building engine | elapsed=80s | stage_age=70s | monitor_seq=2', 80.)
        info = state.model_info('traffic', 80.5)
        self.assertGreater(info['stage_age'], 70)
        self.assertIn('progress unknown', info['health'])
        self.assertFalse(state.can_start(80.5))
        state.update_model('traffic', 'LOADING | Building engine | elapsed=81s | stage_age=71s | monitor_seq=3', 81.)
        self.assertEqual(state.model_changed['traffic'], 80.)
        self.assertIn('STALE', state.model_info('traffic', 87.)['health'])
        state.update_model('traffic', 'FAILED | allocation failed', 88.)
        self.assertFalse(state.checks_ready())
        state.update_model('traffic', 'READY | trial inference=PASS', 90.)
        self.assertEqual(state.model_info('traffic', 100.)['elapsed'], 90.)
        self.assertTrue(state.checks_ready())

    def test_waiting_and_legacy_reports_do_not_claim_a_live_loader(self):
        state = InspectionState(['rgb'])
        self.assertIn('No loader report', state.model_info('rgb', state.started + 5)['health'])
        state.update_model('rgb', 'LOADING | legacy engine load', 10.)
        self.assertIn('progress unknown', state.model_info('rgb', 10.1)['health'])

    def test_python_reporter_runs_during_blocking_load_and_stops_after_ready(self):
        path = SOURCE / 'tunnel_tcn_test/tunnel_tcn_test/loading_status.py'
        spec = importlib.util.spec_from_file_location('_loading_status_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        reports, beat = [], threading.Event()
        def publish(text):
            reports.append(text)
            if 'monitor_seq=2' in text:
                beat.set()
        reporter = module.LoadingStatus(publish, interval=.01)
        try:
            reporter.stage('Building engine')
            self.assertTrue(beat.wait(2.))
            reporter.update('READY | trial inference=PASS')
        finally:
            reporter.close()
        self.assertTrue(reports[-1].startswith('READY | '))
        self.assertFalse(reporter.thread.is_alive())

    def test_cpp_reporter_and_real_backend_constructor_forwarding_compile(self):
        compiler = shutil.which('c++')
        if not compiler:
            self.skipTest('C++ compiler unavailable')
        code = r'''
#include <bev_handoff/model_load_status.hpp>
#include <cassert>
#include <vector>
#include <line_detactor/tensorrt_lane_backend.hpp>
#include <traffic_detection_test/tensorrt_yolox_backend.hpp>
CONSTRUCTORS
int main() {
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<std::string> messages;
  {
    bev_handoff::ModelLoadStatus status([&](const std::string & text) {
      std::lock_guard<std::mutex> lock(mutex);
      messages.push_back(text); changed.notify_all();
    });
    status.update("LOADING | Building engine");
    {
      std::unique_lock<std::mutex> lock(mutex);
      assert(changed.wait_for(lock, std::chrono::seconds(4), [&]{return messages.size()>=2;}));
      assert(messages.back().find("stage_age=1s")!=std::string::npos);
    }
    status.update("READY | trial inference=PASS");
    std::unique_lock<std::mutex> lock(mutex);
    const auto count=messages.size();
    assert(!changed.wait_for(lock, std::chrono::milliseconds(1100), [&]{return messages.size()>count;}));
    assert(messages.back().rfind("READY | trial inference=PASS | elapsed=",0)==0);
  }
}
'''
        constructors = []
        for package, stem, cls in (
            ('line_detactor', 'tensorrt_lane_backend', 'TensorRtLaneBackend'),
            ('traffic_detection_test', 'tensorrt_yolox_backend', 'TensorRtYoloxBackend'),
        ):
            source = (SOURCE / package / 'src' / (stem + '.cpp')).read_text()
            start = source.index('  Impl(') + len('  Impl(')
            end = source.index(')\n  :', start)
            args = source[start:end]
            definition = function(source, cls + '::' + cls + '(')
            constructors.append(f'namespace {package} {{ class {cls}::Impl {{ public: Impl({args}) {{}} }};\n{definition}\n}}')
        code = code.replace('CONSTRUCTORS', '\n'.join(constructors))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'check.cpp'
            path.write_text(code)
            executable = Path(directory) / 'check'
            subprocess.run([compiler, '-std=c++17', '-pthread', '-Wall', '-Wextra',
                            '-Wno-unused-parameter', '-Werror',
                            *['-I' + str(SOURCE / package / 'include') for package in
                              ('bev_handoff', 'line_detactor', 'traffic_detection_test')],
                            str(path), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)
