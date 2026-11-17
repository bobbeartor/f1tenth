"""Rebuild deployment artifacts from the original RGB training directory.

Developer-only dependencies: torch, onnx, onnxruntime, numpy.
Example: python tools/export_rgb_model.py /path/to/brightness_analysis
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import numpy as np
import torch


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('training_dir', type=Path)
    args = parser.parse_args()
    base = args.training_dir.resolve()
    sys.path.insert(0, str(base))
    from rgb_tcn.model import EventTCN
    from rgb_tcn.data import FEATURES, window_at, FeatureExtractor
    from rgb_tcn.training import detect_events
    target = Path(__file__).resolve().parents[1] / 'models'
    target.mkdir(exist_ok=True)
    run_id = '20261023_140042_191723_aug'
    source = base / 'rgb_tcn_runs' / run_id / 'final_model.pt'
    checkpoint = torch.load(source, map_location='cpu', weights_only=False)
    cfg = checkpoint['config']
    assert checkpoint['format'] == 'rgb_point_events_v1'
    assert (cfg['window'], cfg['hz'], cfg['width'], cfg['height']) == (32, 10, 640, 400)
    model = EventTCN(17, cfg)
    model.load_state_dict(checkpoint['state_dict'], strict=True)
    model.eval()
    torch.set_num_threads(1)
    onnx_path = target / 'rgb_point_events_fp32.onnx'
    # Static opset 13 graph for the existing TensorRT 8.6 deployment runtime.
    torch.onnx.export(model, (torch.zeros(1, 17, 32),), str(onnx_path),
                      input_names=['features'], output_names=['logits'],
                      opset_version=13, dynamo=False)
    import onnx
    onnx.checker.check_model(onnx.load(str(onnx_path)))
    shutil.copy2(source, target / 'final_model.pt')
    config = dict(model_id=run_id, model_format=checkpoint['format'], input_shape=[1, 17, 32],
                  output_shape=[1, 2], feature_names=list(FEATURES), history_mask_channel=16,
                  mean=checkpoint['mean'], std=checkpoint['std'], threshold=.5,
                  cooldown_seconds=1., initial_state='unknown', enforce_alternation=True,
                  score_labels=['entry', 'exit'], score_transform='independent_sigmoid',
                  training_config=cfg)
    (target / 'inference_config.json').write_text(json.dumps(config, indent=2)+'\n')
    arrays, items = {}, []
    mean, std = np.asarray(checkpoint['mean'], np.float32), np.asarray(checkpoint['std'], np.float32)
    for index, item_id in enumerate(['20260923_115757__rgb', '20260923_120556__rgb',
                                    '20260923_113130__rgb', '20260923_120108__rgb']):
        with np.load(base / 'rgb_tcn_cache' / (item_id+'.npz')) as data:
            features = data['original']
            meta = json.loads(str(data['metadata']))
        fps = meta['fps']
        times = np.arange(int(np.floor((len(features)-1)/fps*10))+1)/10.
        contexts = np.stack([window_at(features, fps, t, cfg, mean, std) for t in times])
        with torch.no_grad():
            scores = torch.cat([model(x).sigmoid() for x in torch.from_numpy(contexts).split(256)]).numpy()
        arrays[f'features_{index}'] = features
        arrays[f'scores_{index}'] = scores
        arrays[f'times_{index}'] = times
        events = detect_events(times, scores, dict(cfg, enforce_alternation=True, initial_state='unknown'))
        items.append(dict(id=item_id, fps=fps, frames=len(features), samples=len(times),
                          role='external_unlabeled' if index != 1 else 'final_refit_training',
                          expected_events=events))
    import cv2
    rng = np.random.default_rng(20261028)
    images = rng.integers(0, 256, (110, 40, 64), dtype=np.uint8)
    extractor = FeatureExtractor(30., cfg)
    arrays['synthetic_images'] = images
    arrays['synthetic_features'] = np.stack([extractor.push(cv2.resize(im, (640,400), interpolation=cv2.INTER_AREA)) for im in images])
    golden = target / 'golden_replay.npz'
    np.savez_compressed(golden, **arrays)
    provenance = dict(model_id=run_id, source_checkpoint=str(source),
                      checkpoint_sha256=digest(source), onnx_sha256=digest(onnx_path),
                      golden_sha256=digest(golden), config_sha256=digest(target/'inference_config.json'),
                      precision='FP32 (no INT8 quantization)', torch_version=torch.__version__,
                      onnx_version=onnx.__version__, opset=13, replay_items=items)
    (target / 'provenance.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:v for k,v in provenance.items() if k != 'replay_items'}, indent=2))


if __name__ == '__main__':
    main()
