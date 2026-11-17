"""Shared, ROS-independent helpers for the stop-line INT8 PTQ tools."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


CHANNELS = ("left", "right", "stop_line")
EXPECTED_IMAGE_SHAPE = (300, 120, 3)
EXPECTED_TENSOR_SHAPE = (1, 3, 300, 120)


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_bgr(path: Path, expected_sha256: str | None = None) -> np.ndarray:
    encoded = np.fromfile(path, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None or image.shape != EXPECTED_IMAGE_SHAPE:
        raise ValueError(f"Expected W120 H300 BGR image: {path}")
    # Training manifests hash decoded image bytes, not compressed file bytes.
    if expected_sha256 and hashlib.sha256(image.tobytes()).hexdigest() != expected_sha256:
        raise ValueError(f"Decoded image hash differs from manifest: {path}")
    return image


def preprocess_bgr(image: np.ndarray) -> np.ndarray:
    if image.shape != EXPECTED_IMAGE_SHAPE or image.dtype != np.uint8:
        raise ValueError("Expected uint8 BGR image with shape (300,120,3)")
    tensor = np.ascontiguousarray(image[..., ::-1].transpose(2, 0, 1), dtype=np.float32)
    return tensor[None] / 255.0


def load_target(data_root: Path, row: dict, width: int) -> np.ndarray:
    with np.load(data_root / row["mask"], allow_pickle=False) as archive:
        target = archive["masks"].copy()
    if target.shape != (3, 300, 120) or target.dtype != np.uint8:
        raise ValueError(f"Invalid three-channel mask: {row['id']}")
    if not np.isin(target, (0, 1)).all():
        raise ValueError(f"Non-binary mask: {row['id']}")
    if width == 1:
        return target.astype(bool)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (width, width))
    return np.stack([cv2.dilate(channel, kernel) for channel in target]).astype(bool)


def allocate_proportionally(bucket_sizes: dict[str, int], wanted: int) -> dict[str, int]:
    """Largest-remainder allocation without exceeding a bucket."""
    total = sum(bucket_sizes.values())
    if wanted < 0 or wanted > total:
        raise ValueError(f"Cannot select {wanted} rows from {total}")
    if wanted == 0:
        return {name: 0 for name in bucket_sizes}
    exact = {name: wanted * size / total for name, size in bucket_sizes.items()}
    result = {name: min(bucket_sizes[name], int(value)) for name, value in exact.items()}
    remaining = wanted - sum(result.values())
    order = sorted(
        bucket_sizes,
        key=lambda name: (exact[name] - int(exact[name]), bucket_sizes[name], name),
        reverse=True,
    )
    while remaining:
        changed = False
        for name in order:
            if result[name] < bucket_sizes[name]:
                result[name] += 1
                remaining -= 1
                changed = True
                if not remaining:
                    break
        if not changed:
            raise RuntimeError("Proportional allocation stalled")
    return result


def stratified_rows(rows: list[dict], wanted: int, seed: int) -> list[dict]:
    """Sample proportionally by source/group while remaining deterministic."""
    buckets: dict[str, list[dict]] = {}
    for row in rows:
        key = f"{row.get('source', '')}::{row.get('group', '')}"
        buckets.setdefault(key, []).append(row)
    allocation = allocate_proportionally(
        {name: len(values) for name, values in buckets.items()}, wanted
    )
    rng = np.random.default_rng(seed)
    selected = []
    for name in sorted(buckets):
        values = sorted(buckets[name], key=lambda row: row["id"])
        indices = rng.permutation(len(values))[: allocation[name]]
        selected.extend(values[int(index)] for index in indices)
    rng.shuffle(selected)
    return selected


class SegmentationAccumulator:
    def __init__(self, tolerance_px: int):
        self.tolerance_px = tolerance_px
        self.predicted = np.zeros(3, dtype=np.int64)
        self.truth = np.zeros(3, dtype=np.int64)
        self.hit = np.zeros(3, dtype=np.int64)
        self.matched_pred = np.zeros(3, dtype=np.int64)
        self.matched_truth = np.zeros(3, dtype=np.int64)
        self.frames = 0
        self.negative_stop_frames = 0
        self.negative_stop_fp_frames = 0
        self.negative_stop_fp_pixels = 0
        self.positive_stop_frames = 0
        self.positive_stop_frames_with_near_hit = 0

    def update(self, prediction: np.ndarray, truth: np.ndarray) -> None:
        if prediction.shape != (3, 300, 120) or truth.shape != prediction.shape:
            raise ValueError("Expected CxHxW masks with shape (3,300,120)")
        prediction, truth = prediction.astype(bool), truth.astype(bool)
        kernel_size = 2 * self.tolerance_px + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        near_truth = np.stack(
            [cv2.dilate(channel.astype(np.uint8), kernel).astype(bool) for channel in truth]
        )
        near_prediction = np.stack(
            [cv2.dilate(channel.astype(np.uint8), kernel).astype(bool) for channel in prediction]
        )
        self.predicted += prediction.sum((1, 2))
        self.truth += truth.sum((1, 2))
        self.hit += (prediction & truth).sum((1, 2))
        self.matched_pred += (prediction & near_truth).sum((1, 2))
        self.matched_truth += (truth & near_prediction).sum((1, 2))
        stop_truth = bool(truth[2].any())
        stop_count = int(prediction[2].sum())
        if stop_truth:
            self.positive_stop_frames += 1
            self.positive_stop_frames_with_near_hit += bool((prediction[2] & near_truth[2]).any())
        else:
            self.negative_stop_frames += 1
            self.negative_stop_fp_frames += stop_count > 0
            self.negative_stop_fp_pixels += stop_count
        self.frames += 1

    def compute(self) -> dict:
        result = {}
        for index, name in enumerate(CHANNELS):
            predicted = int(self.predicted[index])
            truth = int(self.truth[index])
            hit = int(self.hit[index])
            matched_pred = int(self.matched_pred[index])
            matched_truth = int(self.matched_truth[index])
            tolerance_precision = matched_pred / max(predicted, 1)
            tolerance_recall = matched_truth / max(truth, 1)
            result[name] = {
                "precision": hit / max(predicted, 1),
                "recall": hit / max(truth, 1),
                "f1": 2 * hit / max(predicted + truth, 1),
                "iou": hit / max(predicted + truth - hit, 1),
                "tolerance_f1": 2 * tolerance_precision * tolerance_recall
                / max(tolerance_precision + tolerance_recall, 1e-12),
                "truth_pixels": truth,
                "predicted_pixels": predicted,
            }
        result["lane_macro_f1"] = (result["left"]["f1"] + result["right"]["f1"]) / 2
        result["negative_stop"] = {
            "frames": self.negative_stop_frames,
            "frames_with_any_fp": self.negative_stop_fp_frames,
            "fp_pixels": self.negative_stop_fp_pixels,
            "frame_fp_rate": self.negative_stop_fp_frames / max(self.negative_stop_frames, 1),
        }
        result["positive_stop"] = {
            "frames": self.positive_stop_frames,
            "frames_with_tolerance_hit": self.positive_stop_frames_with_near_hit,
            "frame_hit_rate": self.positive_stop_frames_with_near_hit
            / max(self.positive_stop_frames, 1),
        }
        result["frames"] = self.frames
        return result
