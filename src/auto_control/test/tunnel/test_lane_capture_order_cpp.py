from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class CaptureOrderTests(unittest.TestCase):
    def test_controller_tracks_original_capture_order_per_camera(self):
        compiler = shutil.which('c++')
        if compiler is None:
            self.skipTest('C++ compiler unavailable')
        package = Path(__file__).resolve().parents[2]
        code = r'''
#include "auto_control/lane_capture_order.hpp"
#include <stdexcept>
void require(bool value) {if (!value) {throw std::runtime_error("capture order");}}
int main() {
  auto_control::LaneCaptureOrder order;
  require(order.accept(100,"RGB"));
  require(order.accept(130,"IR"));
  require(order.accept(110,"RGB"));  // Older than IR, newer within RGB.
  require(!order.accept(110,"RGB"));
  require(!order.accept(109,"RGB"));
  require(!order.accept(129,"IR"));
  require(order.accept(131,"IR"));
  require(!order.accept(999,"invalid"));
  require(!order.accept(0,"RGB"));
  require(order.accept(10));  // Ordinary/standalone stream stays independent.
  require(!order.accept(10));
  require(!order.accept(9));
  require(order.accept(11));
}
'''
        with tempfile.TemporaryDirectory() as directory:
            source, binary = Path(directory) / 'order.cpp', Path(directory) / 'order'
            source.write_text(code)
            subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            '-I' + str(package / 'include'), str(source), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
