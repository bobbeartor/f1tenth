"""Create a TensorRT-compatible explicit Q/DQ INT8 PTQ candidate ONNX.

Only Conv nodes are quantized. InstanceNormalization, Resize and the final
three-channel classifier remain floating point. Calibration images are streamed
from a frozen train-only manifest; no labels or validation images are consumed.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import tempfile

import numpy as np
import onnx
import onnxruntime as ort
from onnxruntime.quantization import (
    CalibrationDataReader,
    CalibrationMethod,
    QuantFormat,
    QuantType,
    quant_pre_process,
    quantize_static,
)

from int8_ptq_common import (
    EXPECTED_TENSOR_SHAPE,
    preprocess_bgr,
    read_bgr,
    read_json,
    sha256_file,
    write_json,
)


HERE = Path(__file__).resolve().parent
FINAL_CLASSIFIER = "/model/decoder_classifier/decoder_classifier.1/Conv"


class ImageCalibrationReader(CalibrationDataReader):
    def __init__(self, input_name: str, data_root: Path, samples: list[dict]):
        self.input_name = input_name
        self.data_root = data_root
        self.samples = samples
        self.index = 0

    def get_next(self) -> dict[str, np.ndarray] | None:
        if self.index >= len(self.samples):
            return None
        row = self.samples[self.index]
        self.index += 1
        if self.index % 100 == 0 or self.index == len(self.samples):
            print(f"Calibration input: {self.index}/{len(self.samples)}", flush=True)
        image = read_bgr(self.data_root / row["image"], row["image_sha256"])
        return {self.input_name: preprocess_bgr(image)}

    def rewind(self) -> None:
        self.index = 0


def tensor_shape(value) -> list[int]:
    return [int(d.dim_value) for d in value.type.tensor_type.shape.dim]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--calibration-manifest", type=Path, default=HERE / "int8_calibration_manifest.json")
    parser.add_argument("--config", type=Path, default=HERE / "int8_ptq_config.json")
    parser.add_argument(
        "--calibration-method",
        choices=("minmax", "entropy", "percentile"),
        help="Override the method in int8_ptq_config.json",
    )
    parser.add_argument(
        "--exclude-prefix",
        action="append",
        default=[],
        help="Keep Conv nodes whose ONNX name begins with this prefix in floating point",
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=HERE.parent / "models" / "fast_scnn_stop_line_120x300_batch_1.onnx",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=HERE.parent / "models" / "fast_scnn_stop_line_120x300_batch_1.int8.qdq.onnx",
    )
    args = parser.parse_args()

    source = args.input.resolve()
    output = args.output.resolve()
    data_root = args.data_root.resolve()
    calibration_path = args.calibration_manifest.resolve()
    config = read_json(args.config)
    calibration = read_json(calibration_path)
    samples = calibration.get("samples", [])
    if calibration.get("partition") != "train_calibration_only":
        raise ValueError("Refusing calibration data not marked train_calibration_only")
    if len(samples) != int(config["calibration_count"]):
        raise ValueError("Calibration manifest count differs from PTQ config")
    if len({row["image_sha256"] for row in samples}) != len(samples):
        raise ValueError("Duplicate calibration image")

    model = onnx.load(str(source))
    onnx.checker.check_model(model, full_check=True)
    if len(model.graph.input) != 1 or len(model.graph.output) != 1:
        raise ValueError("Expected one model input and one output")
    if tensor_shape(model.graph.input[0]) != list(EXPECTED_TENSOR_SHAPE):
        raise ValueError("Unexpected source ONNX input shape")
    if tensor_shape(model.graph.output[0]) != list(EXPECTED_TENSOR_SHAPE):
        raise ValueError("Unexpected source ONNX output shape")
    input_name = model.graph.input[0].name
    conv_names = [node.name for node in model.graph.node if node.op_type == "Conv"]
    if FINAL_CLASSIFIER not in conv_names:
        raise ValueError("Expected final classifier Conv was not found")

    methods = {
        "minmax": CalibrationMethod.MinMax,
        "entropy": CalibrationMethod.Entropy,
        "percentile": CalibrationMethod.Percentile,
    }
    method_name = args.calibration_method or str(config["calibration_method"]).lower()
    if method_name not in methods:
        raise ValueError(f"Unsupported calibration method: {method_name}")
    exclude_prefixes = list(config.get("floating_point_exclude_prefixes", []))
    exclude_prefixes.extend(args.exclude_prefix)
    exclusions = [FINAL_CLASSIFIER]
    exclusions.extend(
        name
        for name in conv_names
        if any(name.startswith(prefix) for prefix in exclude_prefixes)
    )
    exclusions = sorted(set(exclusions))
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="line_int8_ptq_") as temporary:
        preprocessed = Path(temporary) / "preprocessed.onnx"
        candidate = Path(temporary) / "candidate.onnx"
        quant_pre_process(
            input_model=str(source),
            output_model_path=str(preprocessed),
            auto_merge=True,
            skip_symbolic_shape=True,
            verbose=1,
        )
        reader = ImageCalibrationReader(input_name, data_root, samples)
        quantize_static(
            model_input=str(preprocessed),
            model_output=str(candidate),
            calibration_data_reader=reader,
            quant_format=QuantFormat.QDQ,
            op_types_to_quantize=["Conv"],
            per_channel=True,
            activation_type=QuantType.QInt8,
            weight_type=QuantType.QInt8,
            nodes_to_exclude=exclusions,
            calibrate_method=methods[method_name],
            extra_options={
                "ActivationSymmetric": True,
                "WeightSymmetric": True,
                # TensorRT 8.6 quantizes Conv bias internally from the
                # activation/weight scales and rejects an explicit INT32 bias
                # DequantizeLinear node on Jetson.
                "QuantizeBias": False,
                "DedicatedQDQPair": False,
            },
        )
        quantized = onnx.load(str(candidate))
        onnx.checker.check_model(quantized, full_check=True)
        if tensor_shape(quantized.graph.input[0]) != list(EXPECTED_TENSOR_SHAPE):
            raise ValueError("Quantized ONNX input shape changed")
        if tensor_shape(quantized.graph.output[0]) != list(EXPECTED_TENSOR_SHAPE):
            raise ValueError("Quantized ONNX output shape changed")
        counts = Counter(node.op_type for node in quantized.graph.node)
        if not counts["QuantizeLinear"] or not counts["DequantizeLinear"]:
            raise ValueError("Explicit Q/DQ nodes were not generated")
        initializer_types = {
            item.name: item.data_type for item in quantized.graph.initializer
        }
        int32_dequantize_nodes = [
            node.name
            for node in quantized.graph.node
            if node.op_type == "DequantizeLinear"
            and node.input
            and initializer_types.get(node.input[0]) == onnx.TensorProto.INT32
        ]
        if int32_dequantize_nodes:
            raise ValueError(
                "TensorRT-incompatible INT32 DequantizeLinear nodes found: "
                f"{int32_dequantize_nodes[:5]}"
            )
        uint8_qdq_nodes = [
            node.name
            for node in quantized.graph.node
            if node.op_type in ("QuantizeLinear", "DequantizeLinear")
            and any(
                initializer_types.get(name) == onnx.TensorProto.UINT8
                for name in node.input
            )
        ]
        if uint8_qdq_nodes:
            raise ValueError(
                "TensorRT-incompatible UINT8 Q/DQ nodes found: "
                f"{uint8_qdq_nodes[:5]}"
            )
        if any(node.domain not in ("", "ai.onnx") for node in quantized.graph.node):
            raise ValueError("Custom ONNX operators are not allowed")
        metadata = {item.key: item.value for item in quantized.metadata_props}
        metadata.update(
            quantization="explicit_qdq_int8_ptq",
            quantized_bias="false",
            calibration_method=method_name,
            calibration_manifest_sha256=sha256_file(calibration_path),
            floating_point_exclusions=json.dumps(exclusions),
        )
        onnx.helper.set_model_props(quantized, metadata)
        onnx.save(quantized, str(candidate))

        session = ort.InferenceSession(str(candidate), providers=["CPUExecutionProvider"])
        zeros = np.zeros(EXPECTED_TENSOR_SHAPE, dtype=np.float32)
        smoke = session.run(None, {session.get_inputs()[0].name: zeros})[0]
        if smoke.shape != EXPECTED_TENSOR_SHAPE or not np.isfinite(smoke).all():
            raise ValueError("Quantized ONNX smoke test failed")
        del session
        candidate.replace(output)

    report = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_onnx": str(source),
        "source_onnx_sha256": sha256_file(source),
        "output_onnx": str(output),
        "output_onnx_sha256": sha256_file(output),
        "format": "explicit Q/DQ INT8 PTQ",
        "calibration_method": method_name,
        "calibration_manifest": str(calibration_path),
        "calibration_manifest_sha256": sha256_file(calibration_path),
        "calibration_count": len(samples),
        "calibration_stop_positive_count": sum(row["has_stop_line"] for row in samples),
        "quantized_op_types": ["Conv"],
        "source_conv_count": len(conv_names),
        "floating_point_exclusions": exclusions,
        "floating_point_exclude_prefixes": exclude_prefixes,
        "qdq_counts": {
            "QuantizeLinear": counts["QuantizeLinear"],
            "DequantizeLinear": counts["DequantizeLinear"],
        },
        "quantized_bias": False,
        "tensorrt_int32_dequantize_count": len(int32_dequantize_nodes),
        "tensorrt_uint8_qdq_count": len(uint8_qdq_nodes),
        "input": list(EXPECTED_TENSOR_SHAPE),
        "output": list(EXPECTED_TENSOR_SHAPE),
        "onnx": onnx.__version__,
        "onnxruntime": ort.__version__,
        "smoke_test": "CPU ONNX Runtime output is finite; this is not a TensorRT latency test",
    }
    write_json(output.with_suffix(".json"), report)
    print(f"Saved {output}")
    print(f"Q/DQ nodes: {report['qdq_counts']}")


if __name__ == "__main__":
    main()
