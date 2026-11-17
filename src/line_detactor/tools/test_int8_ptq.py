from pathlib import Path
import sys
import unittest

import numpy as np
import onnx

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from int8_ptq_common import allocate_proportionally, preprocess_bgr, read_json


class Int8PtqToolsTest(unittest.TestCase):
    def test_proportional_allocation_is_exact_and_bounded(self):
        sizes = {"a": 3, "b": 7, "c": 10}
        result = allocate_proportionally(sizes, 11)
        self.assertEqual(sum(result.values()), 11)
        self.assertTrue(all(0 <= result[name] <= sizes[name] for name in sizes))

    def test_preprocess_contract(self):
        image = np.zeros((300, 120, 3), dtype=np.uint8)
        image[0, 0] = (0, 127, 255)
        tensor = preprocess_bgr(image)
        self.assertEqual(tensor.shape, (1, 3, 300, 120))
        self.assertEqual(tensor.dtype, np.float32)
        np.testing.assert_allclose(tensor[0, :, 0, 0], (1.0, 127 / 255, 0.0))

    def test_frozen_calibration_and_evaluation(self):
        calibration = read_json(HERE / "int8_calibration_manifest.json")
        evaluation = read_json(HERE / "int8_ptq_evaluation.json")
        self.assertEqual(calibration["partition"], "train_calibration_only")
        self.assertEqual(calibration["count"], 512)
        self.assertEqual(calibration["composition"]["stop_positive"], 200)
        self.assertEqual(calibration["composition"]["stop_negative"], 312)
        self.assertEqual(len({row["image_sha256"] for row in calibration["samples"]}), 512)
        self.assertEqual(evaluation["calibration_validation_overlap"], 0)
        self.assertEqual(evaluation["accuracy_verdict"], "pass")
        self.assertTrue(all(evaluation["gate_checks"].values()))

    def test_qdq_artifact_is_symmetric_and_classifier_is_float(self):
        model_path = HERE.parent / "models" / "fast_scnn_stop_line_120x300_batch_1.int8.qdq.onnx"
        model = onnx.load(str(model_path))
        operations = [node.op_type for node in model.graph.node]
        self.assertIn("QuantizeLinear", operations)
        self.assertIn("DequantizeLinear", operations)
        zero_points = [
            onnx.numpy_helper.to_array(value)
            for value in model.graph.initializer
            if "zero_point" in value.name
        ]
        self.assertTrue(zero_points)
        self.assertTrue(all(np.all(value == 0) for value in zero_points))
        classifier = next(
            node
            for node in model.graph.node
            if node.name == "/model/decoder_classifier/decoder_classifier.1/Conv"
        )
        self.assertFalse(any("DequantizeLinear" in name for name in classifier.input))
        decoder_half = [
            node for node in model.graph.node
            if node.op_type == "Conv" and node.name.startswith("/model/decoder_half/")
        ]
        self.assertEqual(len(decoder_half), 4)
        self.assertTrue(
            all(not any("DequantizeLinear" in name for name in node.input) for node in decoder_half)
        )


if __name__ == "__main__":
    unittest.main()
