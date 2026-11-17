"""CPU regression tests; no camera or ROS required."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
import numpy as np
import torch

spec = importlib.util.spec_from_file_location('distance_mlp', Path(__file__).parents[1]/'tools/distance_mlp.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class DistanceTrainingTest(unittest.TestCase):
    def test_features_and_invalid_boxes(self):
        np.testing.assert_allclose(m.features([20, 10, 40, 20], 100, 50, 2),
                                   [.4, .4, .4, .4, .16, 0, 0, 1, 0])
        for box in [[-1, 0, 3, 3], [0, 0, 0, 1], [0, 0, float('nan'), 1], [99, 0, 2, 3]]:
            with self.assertRaises(ValueError):
                m.features(box, 100, 50, 0)

    def test_metrics(self):
        result = m.metrics([.5, 1.5, 3], [.6, 1.3, 3.3])
        self.assertAlmostEqual(result['overall']['mae_m'], .2)
        self.assertAlmostEqual(result['overall']['rmse_m'], np.sqrt(.14/3))
        self.assertIsNone(result['bins']['4-8m']['mae_m'])

    def test_sequence_split_checkpoint_and_runtime_weight_parity(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'samples.jsonl'
            rows = [dict(sequence=s, bbox=[10+j%10, 10, 20+j%5, 30], width=100, height=80,
                         class_id=j%2, distance_m=1+j%5*.1, camera_contract='test_camera')
                    for s in ['one', 'two', 'three'] for j in range(40)]
            path.write_text('\n'.join(json.dumps(r) for r in rows))
            args = SimpleNamespace(data=[str(path)], output=str(Path(tmp)/'model'), seed=42,
                val_fraction=.99, epochs=30, batch_size=32, lr=.01, min_samples=2, max_mae=10, onnx=False)
            m.train(args)
            payload = json.loads((Path(tmp)/'model.json').read_text())
            self.assertFalse(set(payload['train_sequences']) & set(payload['validation_sequences']))
            self.assertEqual(len(payload['train_sequences']), 1)
            self.assertEqual(set(payload['supported_classes']), {0, 1})
            checkpoint = torch.load(Path(tmp)/'model.pt', weights_only=True)
            model = m.DistanceMLP()
            model.load_state_dict(checkpoint['state_dict'])
            f = np.array(m.features(rows[0]['bbox'], 100, 80, 0))
            result = f.copy()
            for i, layer in enumerate(payload['layers']):
                result = np.array(layer['weight']) @ result + layer['bias']
                if i < 2:
                    result = np.maximum(result, 0)
            with torch.no_grad():
                expected = model(torch.tensor(f, dtype=torch.float32)).item()
            self.assertAlmostEqual(result.item(), expected, places=5)
            rows[0]['camera_contract'] = 'other'
            path.write_text('\n'.join(json.dumps(r) for r in rows))
            with self.assertRaises(ValueError):
                m.load_rows([str(path)])
            path.write_text(json.dumps(rows[1]))
            with self.assertRaises(ValueError):
                m.train(args)

if __name__ == '__main__':
    unittest.main()
