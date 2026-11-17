"""Replay delayed source handovers and run their geometry through native control."""
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest
from auto_control_tunnel.selection import LaneSelector
from auto_control_tunnel.path_validation import stamp_ns
from lane_fixture import lane, ns


class HandoverReplayTests(unittest.TestCase):
    def test_delayed_entry_and_exit_keep_real_path_updates_and_native_control(self):
        selector = LaneSelector(validation_mode=True)
        outputs, received = [], []
        schedule = []
        # 80Hz RGB with 80ms sensor-to-result latency. IR becomes valid 220ms
        # after ENTRY, well beyond the controller's 150ms receive timeout.
        for i in range(145):
            t = .90 + i / 80.
            schedule.append((t, 'RGB', lane(t - .08)))
        for i in range(42):
            t = 1.22 + i / 30.
            schedule.append((t, 'IR', lane(t - .01, lateral=.08)))
        schedule += [(1., 'entry', None), (2., 'exit', None)]
        for now, kind, message in sorted(schedule, key=lambda item: item[0]):
            if kind in ('entry', 'exit'):
                selector.event(kind, ns(now), ns(now))
                continue
            # After entry completion RGB is only 5Hz; EXIT/BOTH wakes it to 80.
            if kind == 'RGB' and selector.compute == 'IR':
                continue
            if kind == 'IR' and selector.compute == 'RGB':
                continue
            result = selector.lane(kind, message, ns(now))
            if result is not None:
                outputs.append(result)
                received.append(now)
                self.assertIs(result, message)
                self.assertEqual(result.lane_source, kind)
        self.assertEqual(selector.state, 'OUTSIDE')
        self.assertLess(max(b - a for a, b in zip(received, received[1:])), .10)
        for source in ('RGB', 'IR'):
            stamps = [stamp_ns(m) for m in outputs if m.lane_source == source]
            self.assertTrue(all(b > a for a, b in zip(stamps, stamps[1:])))
        # Exit takes the very first RGB even though its capture predates the
        # previous IR result. The controller has camera-specific ordering.
        self.assertTrue(any(stamp_ns(b) < stamp_ns(a) for a, b in zip(outputs, outputs[1:])))
        self.assertTrue(all(0 <= t - stamp_ns(m) / 1e9 < .15 for t, m in zip(received, outputs)))

        # Exercise real controller geometry/Stanley after float32 ROS point
        # serialization. This detects drift between Python and native rules.
        compiler = shutil.which('c++')
        if compiler is None:
            self.skipTest('C++ compiler unavailable')
        package = Path(__file__).resolve().parents[2]
        source = r'''
#include "auto_control/control_core.hpp"
#include "auto_control/lane_capture_order.hpp"
#include <cmath>
#include <iostream>
#include <vector>
int main() {
  int count, accepted=0;
  std::int64_t stamp; std::string camera;
  auto_control::LaneCaptureOrder order;
  while (std::cin >> count >> stamp >> camera) {
    if (!order.accept(stamp, camera)) {return 4;}
    std::vector<auto_control::Point> points(count);
    for (auto &p: points) {std::cin >> p.x >> p.y;}
    auto path=auto_control::build_ordered_path(points,8,.12,.05,2.20,.15,.14);
    if (!path) {return 2;}
    auto command=auto_control::stanley_control(*path,1.,1.,.2,.1,.523599,.35,.5);
    if (!std::isfinite(command.steering_angle_rad)) {return 3;}
    ++accepted;
  }
  std::cout << accepted;
}
'''
        data = []
        f32 = lambda value: struct.unpack('f', struct.pack('f', value))[0]
        for message in outputs:
            points = [(3. - (f32(p.y) + .5) * .01,
                       .6 - (f32(p.x) - message.padding_left + .5) * .01)
                      for p in message.centerline_points]
            data.append(f'{len(points)} {stamp_ns(message)} {message.lane_source}')
            data.extend(f'{x:.12f} {y:.12f}' for x, y in points)
        with tempfile.TemporaryDirectory() as directory:
            cpp, binary = Path(directory) / 'handover.cpp', Path(directory) / 'handover'
            cpp.write_text(source)
            subprocess.run([compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                            '-I' + str(package / 'include'), str(cpp),
                            str(package / 'src/control_core.cpp'), '-o', str(binary)], check=True,
                           capture_output=True, text=True)
            run = subprocess.run([str(binary)], input='\n'.join(data), text=True, capture_output=True, check=True)
        self.assertEqual(int(run.stdout), len(outputs))


if __name__ == '__main__':
    unittest.main()
