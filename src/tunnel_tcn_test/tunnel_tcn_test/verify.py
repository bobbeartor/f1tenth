"""Replay frozen RGB features and compare deployment to original PyTorch scores/events."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from .features import FeatureStream, EventDecoder, sigmoid
from .backend import TensorRTBackend, OnnxBackend


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['tensorrt', 'onnx'], default='tensorrt')
    parser.add_argument('--cache-dir', default='~/.cache/tunnel_tcn_test')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    share = Path(__file__).resolve().parent.parent
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    except ImportError:
        pass
    else:
        try:
            share = Path(get_package_share_directory('tunnel_tcn_test'))
        except PackageNotFoundError:
            pass
    models = share / 'models'
    provenance = json.loads((models / 'provenance.json').read_text())
    for name, key in [('rgb_point_events_fp32.onnx', 'onnx_sha256'),
                      ('golden_replay.npz', 'golden_sha256'), ('inference_config.json', 'config_sha256')]:
        if hashlib.sha256((models/name).read_bytes()).hexdigest() != provenance[key]:
            raise ValueError(f'{name} checksum mismatch')
    config = json.loads((models / 'inference_config.json').read_text())
    model = models / 'rgb_point_events_fp32.onnx'
    backend = TensorRTBackend(model, args.cache_dir, provenance['onnx_sha256']) if args.backend == 'tensorrt' else OnnxBackend(model)
    rows = []
    try:
        with np.load(models / 'golden_replay.npz') as data:
            for index, item in enumerate(provenance['replay_items']):
                stream = FeatureStream(warmup_seconds=0., mean=config['mean'], std=config['std'])
                decoder = EventDecoder(config['threshold'], config['cooldown_seconds'])
                actual_events, errors, timings = [], [], []
                features, expected, times = data[f'features_{index}'], data[f'scores_{index}'], data[f'times_{index}']
                next_sample = 0
                for frame, values in enumerate(features):
                    timestamp = frame / item['fps']
                    stream.push_features(values, timestamp)
                    while next_sample < len(times) and times[next_sample] <= timestamp+1e-7:
                        # Training uses latest source frame at/before each 10Hz target.
                        # Reference recordings have integer source-FPS/10, so grids coincide.
                        if abs(times[next_sample]-timestamp) > 1e-6:
                            raise ValueError('Replay source grid must align with 10Hz')
                        context = stream._context_at(times[next_sample])
                        started = time.perf_counter()
                        result = sigmoid(backend.infer(context))
                        timings.append((time.perf_counter()-started)*1000)
                        errors.append(float(np.max(np.abs(result-expected[next_sample]))))
                        event = decoder.step(result, float(times[next_sample]))
                        if event:
                            actual_events.append((event['type'], event['time_s']))
                        next_sample += 1
                expected_events = [('entry' if e['label'] == '진입' else 'exit', e['time_s']) for e in item['expected_events']]
                rows.append(dict(id=item['id'], source_frames=len(features), samples=next_sample,
                                 max_score_error=max(errors), event_times_equal=actual_events == expected_events,
                                 actual_events=actual_events, expected_events=expected_events,
                                 inference_plus_sigmoid_p50_ms=float(np.median(timings))))
    finally:
        backend.close()
    success = all(r['max_score_error'] <= .005 and r['event_times_equal'] for r in rows)
    report = dict(backend=args.backend, model_id=provenance['model_id'], passed=success, items=rows,
                  scope='Frozen original features -> checkpoint scaler -> history mask/window -> logits -> sigmoid -> alternating crossings. Warmup disabled for reference parity.',
                  jetson_verified=args.backend == 'tensorrt')
    result = json.dumps(report, ensure_ascii=False, indent=2)+'\n'
    print(result)
    if args.output:
        args.output.write_text(result)
    if not success:
        raise SystemExit('Replay differs from original model')


if __name__ == '__main__':
    main()
