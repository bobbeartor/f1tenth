#!/usr/bin/env python3
"""Summarize runtime paired labels and latency, excluding rejected predictions."""
import argparse
import json
from pathlib import Path
import numpy as np
from distance_mlp import metrics


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument('metrics_file')
    p.add_argument('--output')
    args = p.parse_args()
    rows = [json.loads(x) for x in Path(args.metrics_file).read_text().splitlines() if x.strip()]
    truth, prediction = [], []
    objects = 0
    for row in rows:
        for obj in row['objects']:
            objects += 1
            if obj['distance_m'] is not None and obj['stereo_distance_m'] is not None:
                truth.append(obj['stereo_distance_m'])
                prediction.append(obj['distance_m'])
    result = dict(frames=len(rows), objects=objects, paired_objects=len(truth),
                  distance_error=metrics(truth, prediction), latency={})
    for key in ['yolo_ms', 'mlp_ms', 'yolo_mlp_ms', 'processing_ms', 'capture_to_result_ms']:
        values = [r[key] for r in rows]
        result['latency'][key] = dict(mean=float(np.mean(values)), p95=float(np.percentile(values, 95))) if values else None
    result['final_fps'] = rows[-1]['fps'] if rows else None
    text = json.dumps(result, indent=2, allow_nan=False)
    print(text)
    if args.output:
        Path(args.output).write_text(text)

if __name__ == '__main__':
    main()
