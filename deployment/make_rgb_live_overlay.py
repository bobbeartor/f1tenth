#!/usr/bin/env python3
"""Small, source-and-fitted-model-only update for the RGB live runtime."""
import hashlib
from pathlib import Path
import tarfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    names=[
        'src/multiclass_yolo/src/distance_node.cpp',
        'src/multiclass_yolo/include/multiclass_yolo/distance.hpp',
        'src/multiclass_yolo/include/multiclass_yolo/distance_quality.hpp',
        'src/multiclass_yolo/include/multiclass_yolo/rgb_distance_model.hpp',
        'src/multiclass_yolo/include/multiclass_yolo/distance_live_preview.hpp',
        'src/multiclass_yolo/tools/export_rgb_distance_models.py',
    ]
    names += [str(p.relative_to(ROOT)) for p in sorted((ROOT/'src/yolo_distance_runtime').rglob('*'))
              if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc']
    archive=ROOT/'deployment/rgb-distance-live.tar.gz'
    with tarfile.open(archive,'w:gz') as f:
        for name in names:f.add(ROOT/name,arcname=name,recursive=False)
    (archive.parent/'rgb-distance-live.sha256').write_text(hashlib.sha256(archive.read_bytes()).hexdigest()+'  '+archive.name+'\n')
    (archive.parent/'rgb-distance-live-files.txt').write_text('\n'.join(names)+'\n')
    print(f'{len(names)} files; {archive.stat().st_size:,} bytes; no capture data or YOLO ONNX')

if __name__=='__main__':main()
