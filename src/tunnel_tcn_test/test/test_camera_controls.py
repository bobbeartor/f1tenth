"""Compile the actual driver keyboard handlers without ROS/DepthAI hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


def block(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class ControlsTest(unittest.TestCase):
    def test_projector_decimal_tab_and_field_application(self):
        compiler = shutil.which('clang++') or shutil.which('g++')
        if not compiler:
            self.skipTest('C++ compiler unavailable')
        source = (Path(__file__).resolve().parents[2] / 'ir_camera_driver/src/ir_camera_driver_node.cpp').read_text()
        enum = block(source, 'enum class EditableField') + ';'
        handlers = '\n'.join(block(source, s) for s in [
            'void selectField(', 'std::string * activeInput()', 'bool handleEditingKey('])
        code = '''#include <atomic>
#include <cassert>
#include <string>
class Harness {
public:
ENUM
EditableField editable_field_{EditableField::NONE}, applied{EditableField::NONE};
bool replace_field_on_next_key_{false};
std::string panel_message_, flood_input_{"0.00"}, projector_input_{"1.00"}, exposure_input_{"30000"}, iso_input_{"800"};
std::atomic<bool> control_panel_dirty_{false};
void applyControlInputs(EditableField field) {applied=field;}
HANDLERS
};
int main() {
  Harness h;
  h.selectField(Harness::EditableField::PROJECTOR);
  for(char c : std::string("0.5")) h.handleEditingKey(c);
  assert(h.projector_input_ == "0.5");
  h.handleEditingKey(13);
  assert(h.applied == Harness::EditableField::PROJECTOR);
  assert(h.editable_field_ == Harness::EditableField::NONE);
  for(int i=0;i<4;++i) h.handleEditingKey(9);
  assert(h.editable_field_ == Harness::EditableField::PROJECTOR);
  h.handleEditingKey(9);
  assert(h.editable_field_ == Harness::EditableField::FLOOD);
  h.handleEditingKey('.'); h.handleEditingKey('2');
  assert(h.flood_input_ == ".2");
  h.selectField(Harness::EditableField::ISO);
  h.handleEditingKey('.');
  assert(h.iso_input_ == "800");
  h.handleEditingKey('4'); h.handleEditingKey('0'); h.handleEditingKey('0');
  assert(h.iso_input_ == "400");
}
'''.replace('ENUM', enum).replace('HANDLERS', handlers)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / 'controls.cpp').write_text(code)
            subprocess.run([compiler, '-std=c++17', str(path / 'controls.cpp'), '-o', str(path / 'controls')], check=True, capture_output=True)
            subprocess.run([str(path / 'controls')], check=True, capture_output=True)


if __name__ == '__main__':
    unittest.main()
