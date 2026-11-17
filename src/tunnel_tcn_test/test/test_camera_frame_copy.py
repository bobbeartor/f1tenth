"""Compile the real TCN frame-copy block against a span-like DepthAI buffer."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class CameraFrameCopyTests(unittest.TestCase):
    def test_span_copy_owns_bytes_and_preserves_nv12_padding(self):
        compiler = shutil.which('c++')
        if compiler is None:
            self.skipTest('C++ compiler unavailable')
        source = (Path(__file__).resolve().parents[2] /
                  'camera_driver/src/camera_driver_node.cpp').read_text()
        start = source.index('          message->step=packet->getStride()')
        end = source.index('          tcn_publisher_->publish', start)
        copy_block = source[start:end]
        code = r'''
#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>
struct Span {
  const std::uint8_t * ptr;
  std::size_t length;
  const std::uint8_t * begin() const {return ptr;}
  const std::uint8_t * end() const {return ptr + length;}
};
struct Packet {
  // Width 4, height 2, stride 8: padded Y rows plus padded interleaved UV.
  std::array<std::uint8_t, 24> buffer{};
  unsigned stride{8};
  unsigned getStride() const {return stride;}
  unsigned getWidth() const {return 4;}
  Span getData() const {return {buffer.data(), buffer.size()};}
};
struct Image {unsigned step; std::vector<std::uint8_t> data;};
void require(bool condition) {
  if (!condition) {throw std::runtime_error("TCN NV12 copy regression");}
}
std::unique_ptr<Image> copy(Packet * packet) {
  auto message = std::make_unique<Image>();
COPY_BLOCK
  return message;
}
int main() {
  Packet packet;
  for (unsigned i=0; i<packet.buffer.size(); ++i) {packet.buffer[i]=i+1;}
  const auto message=copy(&packet);
  require(message->step==8 && message->data.size()==24);
  for (unsigned i=0; i<packet.buffer.size(); ++i) {require(message->data[i]==i+1);}
  packet.buffer[0]=99;
  require(message->data[0]==1);  // The ROS message owns its copy.
  packet.stride=0;
  require(copy(&packet)->step==4);  // Missing stride uses frame width.
}
'''.replace('COPY_BLOCK', copy_block)
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory) / 'copy.cpp'
            binary = Path(directory) / 'copy_test'
            file.write_text(code)
            subprocess.run([compiler, '-std=c++17', '-DNDEBUG', '-Wall', '-Wextra',
                            '-Werror', str(file), '-o', str(binary)], check=True,
                           capture_output=True, text=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
