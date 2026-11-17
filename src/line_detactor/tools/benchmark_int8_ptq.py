"""Benchmark FP32 and INT8 Q/DQ ONNX inference on the same preloaded images."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import platform
import time

import numpy as np
import onnxruntime as ort

from int8_ptq_common import preprocess_bgr, read_bgr, read_json, sha256_file, write_json


HERE = Path(__file__).resolve().parent


def create_session(path: Path, threads: int) -> ort.InferenceSession:
    options = ort.SessionOptions()
    options.intra_op_num_threads = threads
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    return ort.InferenceSession(str(path), sess_options=options, providers=["CPUExecutionProvider"])


def summary(nanoseconds: list[int]) -> dict:
    milliseconds = np.asarray(nanoseconds, dtype=np.float64) / 1.0e6
    total_seconds = float(milliseconds.sum() / 1000.0)
    return {
        "runs": len(milliseconds),
        "mean_ms": float(milliseconds.mean()),
        "median_ms": float(np.median(milliseconds)),
        "p95_ms": float(np.percentile(milliseconds, 95)),
        "minimum_ms": float(milliseconds.min()),
        "maximum_ms": float(milliseconds.max()),
        "throughput_fps_from_timed_inference_only": len(milliseconds) / total_seconds,
        "total_timed_seconds": total_seconds,
    }


def run_once(session: ort.InferenceSession, input_name: str, tensor: np.ndarray) -> int:
    started = time.perf_counter_ns()
    output = session.run(None, {input_name: tensor})[0]
    elapsed = time.perf_counter_ns() - started
    if output.shape != (1, 3, 300, 120) or not np.isfinite(output).all():
        raise ValueError("Unexpected/non-finite benchmark output")
    return elapsed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--gallery-index", type=Path, required=True)
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
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--warmup-runs", type=int, default=100)
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--output", type=Path, default=HERE / "int8_ptq_cpu_benchmark.json")
    args = parser.parse_args()
    if args.threads < 1 or args.warmup_runs < 1 or args.repetitions < 1:
        raise ValueError("threads, warmup-runs and repetitions must be positive")

    data_root = args.data_root.resolve()
    gallery_path = args.gallery_index.resolve()
    gallery = read_json(gallery_path)
    samples = gallery.get("samples", [])
    if not samples:
        raise ValueError("Gallery index has no samples")
    validation_rows = {
        row["id"]: row for row in read_json(Path(gallery["validation_manifest"])).get("samples", [])
    }
    tensors = []
    for row in samples:
        if row["id"] not in validation_rows:
            raise ValueError(f"Gallery sample is absent from validation manifest: {row['id']}")
        image_path = data_root / validation_rows[row["id"]]["image"]
        tensors.append(preprocess_bgr(read_bgr(image_path, row["image_sha256"])))

    reference_path = args.reference.resolve()
    candidate_path = args.candidate.resolve()
    reference = create_session(reference_path, args.threads)
    candidate = create_session(candidate_path, args.threads)
    reference_input = reference.get_inputs()[0].name
    candidate_input = candidate.get_inputs()[0].name
    sessions = {
        "fp32": (reference, reference_input),
        "int8_qdq": (candidate, candidate_input),
    }
    for name, (session, input_name) in sessions.items():
        for index in range(args.warmup_runs):
            session.run(None, {input_name: tensors[index % len(tensors)]})
        print(f"Warmup complete: {name} {args.warmup_runs} runs", flush=True)

    timings = {name: [] for name in sessions}
    for repetition in range(args.repetitions):
        order = ("fp32", "int8_qdq") if repetition % 2 == 0 else ("int8_qdq", "fp32")
        for tensor in tensors:
            for name in order:
                session, input_name = sessions[name]
                timings[name].append(run_once(session, input_name, tensor))
        print(f"Timed repetitions: {repetition + 1}/{args.repetitions}", flush=True)

    results = {name: summary(values) for name, values in timings.items()}
    mean_speedup = results["fp32"]["mean_ms"] / results["int8_qdq"]["mean_ms"]
    median_speedup = results["fp32"]["median_ms"] / results["int8_qdq"]["median_ms"]
    report = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "ONNX Runtime CPU model inference only; preloaded tensors; not Jetson/TensorRT",
        "environment": {
            "platform": platform.platform(),
            "processor": platform.processor(),
            "logical_cpu_count": os.cpu_count(),
            "onnxruntime": ort.__version__,
            "providers": ort.get_available_providers(),
            "selected_provider": "CPUExecutionProvider",
            "intra_op_threads": args.threads,
            "inter_op_threads": 1,
        },
        "input": {
            "gallery_index": str(gallery_path),
            "gallery_index_sha256": sha256_file(gallery_path),
            "unique_images": len(tensors),
            "shape": [1, 3, 300, 120],
            "preprocessing_and_file_io_included": False,
        },
        "warmup_runs_per_model": args.warmup_runs,
        "repetitions_per_image": args.repetitions,
        "models": {
            "fp32": {
                "path": str(reference_path),
                "sha256": sha256_file(reference_path),
                "file_bytes": reference_path.stat().st_size,
            },
            "int8_qdq": {
                "path": str(candidate_path),
                "sha256": sha256_file(candidate_path),
                "file_bytes": candidate_path.stat().st_size,
            },
        },
        "results": results,
        "comparison": {
            "mean_speedup_fp32_over_int8": mean_speedup,
            "median_speedup_fp32_over_int8": median_speedup,
            "mean_latency_reduction_percent": (1.0 - 1.0 / mean_speedup) * 100.0,
            "model_file_size_reduction_percent": (
                1.0 - candidate_path.stat().st_size / reference_path.stat().st_size
            )
            * 100.0,
        },
        "decision_note": "Use target-Jetson TensorRT execution timing for the deployment decision.",
    }
    write_json(args.output, report)
    print(f"Saved {args.output.resolve()}")
    print(
        "FP32 mean={:.4f}ms, INT8 mean={:.4f}ms, speedup={:.3f}x".format(
            results["fp32"]["mean_ms"], results["int8_qdq"]["mean_ms"], mean_speedup
        )
    )


if __name__ == "__main__":
    main()
