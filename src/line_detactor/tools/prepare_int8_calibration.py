"""Create a reproducible, train-only calibration manifest for INT8 PTQ."""
from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

from int8_ptq_common import read_bgr, read_json, sha256_file, stratified_rows, write_json


HERE = Path(__file__).resolve().parent


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True, help="line_detector training-data root")
    parser.add_argument("--train-manifest", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=HERE / "int8_ptq_config.json")
    parser.add_argument("--output", type=Path, default=HERE / "int8_calibration_manifest.json")
    parser.add_argument("--verify-images", action="store_true", help="Decode and hash all selected images")
    args = parser.parse_args()

    data_root = args.data_root.resolve()
    train_manifest = args.train_manifest.resolve()
    config = read_json(args.config)
    manifest = read_json(train_manifest)
    rows = manifest.get("samples", [])
    if manifest.get("partition") != "train" or not rows:
        raise ValueError("Calibration source must be a non-empty train manifest")
    if any(row.get("partition") != "train" for row in rows):
        raise ValueError("Calibration manifest contains a non-train sample")
    if len({row["image_sha256"] for row in rows}) != len(rows):
        raise ValueError("Duplicate decoded images in train manifest")

    total = int(config["calibration_count"])
    positive_count = int(config["calibration_stop_positive_count"])
    positive = [row for row in rows if row.get("has_stop_line") is True]
    negative = [row for row in rows if row.get("has_stop_line") is False]
    if len(positive) < positive_count or len(negative) < total - positive_count:
        raise ValueError("Requested calibration composition is unavailable")
    selected = stratified_rows(positive, positive_count, int(config["seed"]))
    selected += stratified_rows(negative, total - positive_count, int(config["seed"]) + 1)
    selected = stratified_rows(selected, total, int(config["seed"]) + 2)

    compact = []
    for index, row in enumerate(selected, 1):
        path = data_root / row["image"]
        if not path.is_file():
            raise FileNotFoundError(path)
        if args.verify_images:
            read_bgr(path, row["image_sha256"])
        compact.append(
            {
                key: row[key]
                for key in ("id", "image", "image_sha256", "source", "group", "has_stop_line")
            }
        )
        if args.verify_images and (index % 100 == 0 or index == len(selected)):
            print(f"Verified calibration images: {index}/{len(selected)}", flush=True)

    if len({row["image_sha256"] for row in compact}) != total:
        raise RuntimeError("Calibration selection is not unique")
    composition = {
        "stop_positive": sum(row["has_stop_line"] for row in compact),
        "stop_negative": sum(not row["has_stop_line"] for row in compact),
        "by_source": dict(sorted(Counter(row["source"] for row in compact).items())),
        "by_group": dict(sorted(Counter(row["group"] for row in compact).items())),
    }
    payload = {
        "schema_version": 1,
        "partition": "train_calibration_only",
        "selection": "deterministic proportional source/group stratification",
        "seed": config["seed"],
        "source_manifest": str(train_manifest),
        "source_manifest_sha256": sha256_file(train_manifest),
        "data_root_recorded_for_audit": str(data_root),
        "count": total,
        "composition": composition,
        "samples": compact,
    }
    write_json(args.output, payload)
    print(f"Saved {args.output.resolve()}")
    print(composition)


if __name__ == "__main__":
    main()
