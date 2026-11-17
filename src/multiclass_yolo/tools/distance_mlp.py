#!/usr/bin/env python3
"""Sequence-held-out distance training. Runtime JSON uses identical row-major weights."""
import argparse
import json
from pathlib import Path
import numpy as np
import torch
from torch import nn

CLASSES = ['con', 'truck', 'sedan', 'traffic_light']


def features(box, width, height, class_id):
    x, y, w, h = map(float, box)
    if (width <= 0 or height <= 0 or class_id not in range(4) or
            not np.isfinite([x, y, w, h]).all() or
            x < 0 or y < 0 or w <= 0 or h <= 0 or
            x+w > width+0.01 or y+h > height+0.01):
        raise ValueError('Invalid original-image bounding box/class')
    return [(x+w/2)/width, (y+h/2)/height, w/width, h/height,
            w*h/(width*height)] + [float(i == class_id) for i in range(4)]


class DistanceMLP(nn.Sequential):
    def __init__(self):
        super().__init__(nn.Linear(9, 64), nn.ReLU(), nn.Linear(64, 32),
                         nn.ReLU(), nn.Linear(32, 1))


def metrics(target, prediction):
    target, prediction = np.asarray(target), np.asarray(prediction)
    def score(mask):
        error = prediction[mask]-target[mask]
        return dict(count=len(error), mae_m=float(np.abs(error).mean()) if len(error) else None,
                    rmse_m=float(np.sqrt(np.mean(error**2))) if len(error) else None)
    return dict(overall=score(np.ones(len(target), dtype=bool)), bins={
        f'{lo}-{hi}m': score((target >= lo) & (target < hi))
        for lo, hi in [(0, 1), (1, 2), (2, 4), (4, 8), (8, float('inf'))]})


def load_rows(paths):
    rows = [json.loads(line) for path in paths for line in Path(path).read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError('Empty dataset')
    contracts = {r['camera_contract'] for r in rows}
    if len(contracts) != 1:
        raise ValueError('Camera contracts differ; train separately for each geometry')
    for r in rows:
        features(r['bbox'], r['width'], r['height'], r['class_id'])
        if not np.isfinite(r['distance_m']) or r['distance_m'] <= 0 or not r['sequence']:
            raise ValueError('Invalid label/sequence')
    return rows


def train(args):
    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    rows = load_rows(args.data)
    sequences = sorted({r['sequence'] for r in rows})
    if len(sequences) < 2:
        raise ValueError('At least two independent capture sequences required')
    rng = np.random.default_rng(args.seed)
    rng.shuffle(sequences)
    val_sequences = set(sequences[:min(len(sequences)-1, max(1, round(len(sequences)*args.val_fraction)))])
    train_rows = [r for r in rows if r['sequence'] not in val_sequences]
    val_rows = [r for r in rows if r['sequence'] in val_sequences]
    def tensors(part):
        return (torch.tensor([features(r['bbox'], r['width'], r['height'], r['class_id']) for r in part]),
                torch.tensor([[r['distance_m']] for r in part]))
    x, y = tensors(train_rows)
    vx, vy = tensors(val_rows)
    model = DistanceMLP()
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    best, state = float('inf'), None
    for _ in range(args.epochs):
        model.train()
        for indices in torch.randperm(len(x)).split(args.batch_size):
            opt.zero_grad()
            loss = nn.functional.huber_loss(model(x[indices]), y[indices], delta=1.0)
            loss.backward()
            opt.step()
        model.eval()
        with torch.no_grad():
            error = (model(vx)-vy).abs().mean().item()
        if error < best:
            best, state = error, {k: v.detach().clone() for k, v in model.state_dict().items()}
    if state is None:
        raise ValueError('Training diverged; no finite validation result')
    model.load_state_dict(state)
    model.eval()
    with torch.no_grad():
        predictions = model(vx).numpy().ravel()
    report = metrics(vy.numpy().ravel(), predictions)
    report['classes'] = {
        CLASSES[c]: metrics(vy.numpy().ravel()[mask], predictions[mask])
        for c in range(4)
        for mask in [np.array([r['class_id'] == c for r in val_rows])]
    }
    # Each admitted class must have independent held-out evidence and positive predictions.
    supported = []
    for class_id in range(4):
        tr = [r for r in train_rows if r['class_id'] == class_id]
        mask = np.array([r['class_id'] == class_id for r in val_rows])
        accepted = (len(tr) >= args.min_samples and mask.sum() >= args.min_samples and
                    np.isfinite(predictions[mask]).all() and (predictions[mask] > 0).all() and
                    np.abs(predictions[mask]-vy.numpy().ravel()[mask]).mean() <= args.max_mae)
        if accepted:
            supported.append(class_id)
    domains = {}
    for c in supported:
        part = [r for r in train_rows if r['class_id'] == c]
        cx = np.array([features(r['bbox'], r['width'], r['height'], c)[:5] for r in part])
        domains[str(c)] = dict(min=cx.min(0).tolist(), max=cx.max(0).tolist(),
                              distance_min=min(r['distance_m'] for r in part),
                              distance_max=max(r['distance_m'] for r in part))
    payload = dict(version=1, classes=CLASSES, feature_order=['cx','cy','w','h','area','class_one_hot'],
                   distance_definition='rgb_optical_z_m', camera_contract=rows[0]['camera_contract'],
                   supported_classes=supported, domains=domains,
                   train_sequences=sorted(set(sequences)-val_sequences),
                   validation_sequences=sorted(val_sequences), metrics=report,
                   validation_policy=dict(min_samples=args.min_samples, max_mae_m=args.max_mae),
                   layers=[dict(weight=layer.weight.detach().tolist(), bias=layer.bias.detach().tolist())
                           for layer in model if isinstance(layer, nn.Linear)])
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    torch.save(dict(state_dict=state, metadata=payload), out.with_suffix('.pt'))
    out.with_suffix('.json').write_text(json.dumps(payload, indent=2, allow_nan=False))
    print(json.dumps(dict(metrics=report, supported_classes=supported), indent=2))
    if args.onnx:
        torch.onnx.export(model, torch.zeros(1, 9), str(out.with_suffix('.onnx')),
                          input_names=['features'], output_names=['distance_m'],
                          dynamic_axes={'features': {0: 'objects'}, 'distance_m': {0: 'objects'}},
                          opset_version=13, dynamo=False)


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument('data', nargs='+', help='collected samples.jsonl files')
    p.add_argument('--output', required=True)
    p.add_argument('--epochs', type=int, default=200)
    p.add_argument('--batch-size', type=int, default=256)
    p.add_argument('--lr', type=float, default=0.001)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--val-fraction', type=float, default=0.2)
    p.add_argument('--min-samples', type=int, default=30)
    p.add_argument('--max-mae', type=float, default=0.3)
    p.add_argument('--onnx', action='store_true')
    a = p.parse_args()
    if not (0 < a.val_fraction < 1 and a.epochs > 0 and a.batch_size > 0 and
            a.lr > 0 and a.min_samples > 0 and a.max_mae > 0):
        p.error('Invalid training/validation settings')
    train(a)

if __name__ == '__main__':
    main()
