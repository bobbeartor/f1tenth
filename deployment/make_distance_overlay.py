#!/usr/bin/env python3
"""Package only distance-feature edits; leave existing YOLO/models/vehicle files intact."""
from pathlib import Path
import hashlib
import tarfile

ROOT = Path(__file__).resolve().parents[1]
FILES = [
    'src/yolo_distance_runtime/CMakeLists.txt',
    'src/yolo_distance_runtime/package.xml',
    'src/multiclass_yolo/src/distance_node.cpp',
    'src/multiclass_yolo/include/multiclass_yolo/distance.hpp',
    'src/multiclass_yolo/include/multiclass_yolo/distance_quality.hpp',
    'src/multiclass_yolo/tools/distance_mlp.py',
    'src/multiclass_yolo/tools/evaluate_distance.py',
    'src/multiclass_yolo/tools/requirements-distance.txt',
    'src/multiclass_yolo/test/distance_quality_test.cpp',
    'src/multiclass_yolo/test/test_distance_training.py',
    'deployment/README.md',
]


def main():
    directory = ROOT / 'deployment'
    archive = directory / 'distance-overlay.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        for name in FILES:
            tar.add(ROOT/name, arcname=name, recursive=False)
    (directory/'distance-overlay.sha256').write_text(
        hashlib.sha256(archive.read_bytes()).hexdigest()+'  '+archive.name+'\n')
    (directory/'distance-overlay-files.txt').write_text('\n'.join(FILES)+'\n')
    print(f'{archive.name}: {len(FILES)} files, {archive.stat().st_size:,} bytes')

if __name__ == '__main__':
    main()
