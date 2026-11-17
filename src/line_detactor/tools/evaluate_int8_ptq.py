"""Compare FP32 and INT8 Q/DQ ONNX accuracy on a labeled validation manifest."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys

import numpy as np
import onnxruntime as ort

from int8_ptq_common import (
    CHANNELS,
    SegmentationAccumulator,
    read_json,
    sha256_file,
    write_json,
)


HERE = Path(__file__).resolve().parent


def create_session(path: Path) -> ort.InferenceSession:
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    options.inter_op_num_threads = 1
    return ort.InferenceSession(str(path), sess_options=options, providers=["CPUExecutionProvider"])


def metric_delta(candidate: dict, reference: dict) -> dict:
    return {
        "stop_f1": candidate["stop_line"]["f1"] - reference["stop_line"]["f1"],
        "lane_macro_f1": candidate["lane_macro_f1"] - reference["lane_macro_f1"],
        "negative_stop_frame_fp_rate": candidate["negative_stop"]["frame_fp_rate"]
        - reference["negative_stop"]["frame_fp_rate"],
        "positive_stop_frame_hit_rate": candidate["positive_stop"]["frame_hit_rate"]
        - reference["positive_stop"]["frame_hit_rate"],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--validation-manifest", type=Path, required=True)
    parser.add_argument("--calibration-manifest", type=Path, default=HERE / "int8_calibration_manifest.json")
    parser.add_argument("--config", type=Path, default=HERE / "int8_ptq_config.json")
    parser.add_argument(
        "--reference",
        type=Path,
        default=HERE.parent / "models" / "fast_scnn_stop_line_120x300_batch_1.onnx",
    )
    parser.add_argument(
        "--candidate",
        type=Path,
        default=HERE.parent / "models" / "fast_scnn_stop_line_120x300_batch_1.int8.qdq.onnx",
    )
    parser.add_argument("--output", type=Path, default=HERE / "int8_ptq_evaluation.json")
    args = parser.parse_args()

    data_root = args.data_root.resolve()
    validation_path = args.validation_manifest.resolve()
    reference_path = args.reference.resolve()
    candidate_path = args.candidate.resolve()
    config = read_json(args.config)
    validation = read_json(validation_path)
    rows = validation.get("samples", [])
    if validation.get("partition") != "val" or not rows:
        raise ValueError("Evaluation requires a non-empty val manifest")
    if any(row.get("partition") != "val" for row in rows):
        raise ValueError("Validation manifest contains a non-val sample")
    calibration = read_json(args.calibration_manifest)
    calibration_hashes = {row["image_sha256"] for row in calibration.get("samples", [])}
    validation_hashes = {row["image_sha256"] for row in rows}
    if calibration_hashes & validation_hashes:
        raise ValueError("Calibration and validation images overlap")

    stop_training = data_root / "training" / "highres" / "stop_line_0910"
    for module_path in (stop_training, stop_training.parent, data_root / "training"):
        sys.path.insert(0, str(module_path))
    from data_0910 import StopDataset

    training_config = read_json(stop_training / "train_config_0910.json")
    if float(training_config["threshold"]) != float(config["threshold"]):
        raise ValueError("PTQ threshold differs from the trained-model threshold")
    if int(training_config["target_width"]) != int(config["target_width"]):
        raise ValueError("PTQ target width differs from training")
    dataset = StopDataset(
        rows,
        "val",
        training_config,
        read_json(stop_training / "occlusion_config.json"),
    )
    dataset.set_epoch(0)

    reference_session = create_session(reference_path)
    candidate_session = create_session(candidate_path)
    reference_input = reference_session.get_inputs()[0].name
    candidate_input = candidate_session.get_inputs()[0].name
    threshold_logit = float(np.log(config["threshold"] / (1.0 - config["threshold"])))
    modes = ("clean", "occluded")
    reference_metrics = {
        mode: SegmentationAccumulator(int(config["tolerance_px"])) for mode in modes
    }
    candidate_metrics = {
        mode: SegmentationAccumulator(int(config["tolerance_px"])) for mode in modes
    }
    pairwise = {
        mode: {
            "mask_difference_pixels": np.zeros(3, dtype=np.int64),
            "maximum_absolute_logit_error": 0.0,
            "stop_frame_decision_changes": 0,
        }
        for mode in modes
    }

    for index in range(len(dataset)):
        item = dataset[index]
        truth = item["target"].numpy() > 0.5
        inputs = {
            "clean": item["clean_image"].numpy()[None],
            "occluded": item["image"].numpy()[None],
        }
        for mode, tensor in inputs.items():
            reference_logits = reference_session.run(None, {reference_input: tensor})[0][0]
            candidate_logits = candidate_session.run(None, {candidate_input: tensor})[0][0]
            if not np.isfinite(reference_logits).all() or not np.isfinite(candidate_logits).all():
                raise ValueError(f"Non-finite {mode} output: {item['id']}")
            reference_mask = reference_logits >= threshold_logit
            candidate_mask = candidate_logits >= threshold_logit
            reference_metrics[mode].update(reference_mask, truth)
            candidate_metrics[mode].update(candidate_mask, truth)
            details = pairwise[mode]
            details["mask_difference_pixels"] += (reference_mask != candidate_mask).sum((1, 2))
            details["maximum_absolute_logit_error"] = max(
                details["maximum_absolute_logit_error"],
                float(np.max(np.abs(reference_logits - candidate_logits))),
            )
            details["stop_frame_decision_changes"] += (
                bool(reference_mask[2].any()) != bool(candidate_mask[2].any())
            )
        if (index + 1) % 50 == 0 or index + 1 == len(dataset):
            print(f"Evaluated clean + occluded images: {index + 1}/{len(dataset)}", flush=True)

    reference_result = {mode: reference_metrics[mode].compute() for mode in modes}
    candidate_result = {mode: candidate_metrics[mode].compute() for mode in modes}
    delta = {
        mode: metric_delta(candidate_result[mode], reference_result[mode]) for mode in modes
    }
    for mode in modes:
        pairwise[mode]["mask_difference_pixels"] = dict(
            zip(CHANNELS, pairwise[mode]["mask_difference_pixels"].tolist())
        )
    gates = config["gates"]
    checks = {
        "stop_f1_all_modes": min(delta[mode]["stop_f1"] for mode in modes)
        >= -float(gates["maximum_stop_f1_drop"]),
        "lane_macro_f1_all_modes": min(delta[mode]["lane_macro_f1"] for mode in modes)
        >= -float(gates["maximum_lane_macro_f1_drop"]),
        "negative_stop_frame_fp_rate_all_modes": max(
            delta[mode]["negative_stop_frame_fp_rate"] for mode in modes
        )
        <= float(gates["maximum_negative_stop_frame_fp_rate_increase"]),
        "positive_stop_frame_hit_rate_all_modes": min(
            delta[mode]["positive_stop_frame_hit_rate"] for mode in modes
        )
        >= -float(gates["maximum_positive_stop_frame_hit_rate_drop"]),
    }
    report = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "fixed clean and deterministic synthetic-occlusion validation; quantization A/B only",
        "validation_manifest": str(validation_path),
        "validation_manifest_sha256": sha256_file(validation_path),
        "calibration_manifest_sha256": sha256_file(args.calibration_manifest),
        "calibration_validation_overlap": 0,
        "reference": {
            "path": str(reference_path),
            "sha256": sha256_file(reference_path),
            "metrics": reference_result,
        },
        "candidate": {
            "path": str(candidate_path),
            "sha256": sha256_file(candidate_path),
            "metrics": candidate_result,
        },
        "candidate_minus_reference": delta,
        "pairwise": pairwise,
        "gates": gates,
        "gate_checks": checks,
        "accuracy_verdict": "pass" if all(checks.values()) else "fail",
        "latency_verdict": "not_tested; benchmark the TensorRT engine on the target Jetson",
    }
    write_json(args.output, report)
    print(f"Saved {args.output.resolve()}")
    print(f"Accuracy verdict: {report['accuracy_verdict']} | delta={delta}")


if __name__ == "__main__":
    main()
