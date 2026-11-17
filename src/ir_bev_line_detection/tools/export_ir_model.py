"""Reuse the production offline exporter for a stereo-only best.pt checkpoint.

Example from the repository root:
python src/ir_bev_line_detection/tools/export_ir_model.py --checkpoint /path/best.pt \
    --validation-images /path/to/IR/validation/images
Only validation images should be supplied, never the final-test session.
"""
from pathlib import Path
import runpy
import sys


def main():
    package = Path(__file__).resolve().parents[1]
    # Works both in src/<package>/tools and share/<package>/tools.
    exporter = package.parent / 'line_detactor/tools/export_stop_line_onnx.py'
    if not exporter.is_file():
        raise FileNotFoundError(f'Production exporter not found: {exporter}')
    if not any(arg == '--checkpoint' or arg.startswith('--checkpoint=') for arg in sys.argv[1:]):
        raise ValueError('Supply the stereo-only checkpoint explicitly with --checkpoint')
    arguments = sys.argv[1:]
    if not any(arg == '--output' or arg.startswith('--output=') for arg in arguments):
        arguments += ['--output', str(package.parent / 'line_detactor/models/ir_stereo_bev_120x300.onnx')]
    sys.argv = [str(exporter), *arguments]
    runpy.run_path(str(exporter), run_name='__main__')


if __name__ == '__main__':
    main()
