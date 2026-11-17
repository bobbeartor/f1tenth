"""Render a balanced visual FP32-versus-INT8 validation gallery."""
from __future__ import annotations

import argparse
from collections import Counter
import html
from pathlib import Path
import sys

import cv2
import numpy as np
import onnxruntime as ort

from int8_ptq_common import read_json, sha256_file, stratified_rows, write_json


HERE = Path(__file__).resolve().parent
CHANNEL_COLORS = ((255, 255, 0), (0, 165, 255), (255, 0, 255))


def create_session(path: Path) -> ort.InferenceSession:
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    options.inter_op_num_threads = 1
    return ort.InferenceSession(str(path), sess_options=options, providers=["CPUExecutionProvider"])


def bgr(tensor) -> np.ndarray:
    array = tensor.numpy() if hasattr(tensor, "numpy") else np.asarray(tensor)
    return np.ascontiguousarray(
        (array.transpose(1, 2, 0)[..., ::-1] * 255.0).clip(0, 255).astype(np.uint8)
    )


def overlay(image: np.ndarray, masks: np.ndarray) -> np.ndarray:
    result = image.copy()
    for mask, color in zip(masks, CHANNEL_COLORS):
        selected = mask.astype(bool)
        result[selected] = (
            0.20 * result[selected] + 0.80 * np.asarray(color, dtype=np.float32)
        ).astype(np.uint8)
    return result


def panel(image: np.ndarray, title: str, detail: str = "") -> np.ndarray:
    result = cv2.copyMakeBorder(image, 38, 0, 0, 0, cv2.BORDER_CONSTANT)
    cv2.putText(result, title, (3, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255, 255, 255), 1)
    if detail:
        cv2.putText(result, detail, (3, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.23, (200, 200, 200), 1)
    return result


def save_png(path: Path, image: np.ndarray) -> None:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise OSError(f"PNG encoding failed: {path}")
    encoded.tofile(path)


def infer(session: ort.InferenceSession, input_name: str, tensor: np.ndarray) -> np.ndarray:
    logits = session.run(None, {input_name: tensor[None]})[0][0]
    if logits.shape != (3, 300, 120) or not np.isfinite(logits).all():
        raise ValueError("Unexpected/non-finite ONNX output")
    return logits


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--validation-manifest", type=Path, required=True)
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
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--stop-positive-count", type=int, default=50)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    data_root = args.data_root.resolve()
    validation_path = args.validation_manifest.resolve()
    output = args.output.resolve()
    config = read_json(args.config)
    manifest = read_json(validation_path)
    rows = manifest.get("samples", [])
    if manifest.get("partition") != "val" or not rows:
        raise ValueError("A non-empty val manifest is required")
    if not 0 <= args.stop_positive_count <= args.count <= len(rows):
        raise ValueError("Invalid gallery counts")
    positive = [row for row in rows if row.get("has_stop_line") is True]
    negative = [row for row in rows if row.get("has_stop_line") is False]
    negative_count = args.count - args.stop_positive_count
    selected = stratified_rows(positive, args.stop_positive_count, int(config["seed"]) + 100)
    selected += stratified_rows(negative, negative_count, int(config["seed"]) + 101)
    selected = stratified_rows(selected, args.count, int(config["seed"]) + 102)
    selected_ids = {row["id"] for row in selected}
    if len(selected_ids) != args.count:
        raise RuntimeError("Gallery selection is not unique")

    stop_training = data_root / "training" / "highres" / "stop_line_0910"
    for module_path in (stop_training, stop_training.parent, data_root / "training"):
        sys.path.insert(0, str(module_path))
    from data_0910 import StopDataset

    training_config = read_json(stop_training / "train_config_0910.json")
    dataset = StopDataset(
        rows, "val", training_config, read_json(stop_training / "occlusion_config.json")
    )
    dataset.set_epoch(0)
    row_index = {row["id"]: index for index, row in enumerate(rows)}

    reference_path = args.reference.resolve()
    candidate_path = args.candidate.resolve()
    reference = create_session(reference_path)
    candidate = create_session(candidate_path)
    reference_input = reference.get_inputs()[0].name
    candidate_input = candidate.get_inputs()[0].name
    threshold = float(np.log(config["threshold"] / (1.0 - config["threshold"])))

    if output.exists():
        raise FileExistsError(f"Output already exists: {output}")
    images = output / "images"
    images.mkdir(parents=True)
    records = []
    cards = []
    for gallery_index, row in enumerate(selected):
        item = dataset[row_index[row["id"]]]
        truth = item["target"].numpy() > 0.5
        clean_tensor = item["clean_image"].numpy()
        occluded_tensor = item["image"].numpy()
        clean_image = bgr(item["clean_image"])
        occluded_image = bgr(item["image"])
        outputs = {}
        for mode, tensor in (("clean", clean_tensor), ("occluded", occluded_tensor)):
            fp32_logits = infer(reference, reference_input, tensor)
            int8_logits = infer(candidate, candidate_input, tensor)
            outputs[mode] = (fp32_logits >= threshold, int8_logits >= threshold)

        clean_fp32, clean_int8 = outputs["clean"]
        occluded_fp32, occluded_int8 = outputs["occluded"]
        truth_detail = "L/R/S=" + "/".join(str(int(channel.sum())) for channel in truth)
        clean_diff = (clean_fp32 != clean_int8).sum((1, 2)).astype(int).tolist()
        occluded_diff = (occluded_fp32 != occluded_int8).sum((1, 2)).astype(int).tolist()
        top = np.hstack(
            [
                panel(clean_image, "CLEAN INPUT", row["source"]),
                panel(overlay(clean_image, truth), "GROUND TRUTH", truth_detail),
                panel(overlay(clean_image, clean_fp32), "FP32 CLEAN"),
                panel(overlay(clean_image, clean_int8), "INT8 CLEAN", f"diff L/R/S={clean_diff}"),
            ]
        )
        bottom = np.hstack(
            [
                panel(occluded_image, "OCCLUDED INPUT", row.get("group", "")),
                panel(overlay(occluded_image, truth), "GT + OCCLUSION"),
                panel(overlay(occluded_image, occluded_fp32), "FP32 OCCLUDED"),
                panel(
                    overlay(occluded_image, occluded_int8),
                    "INT8 OCCLUDED",
                    f"diff L/R/S={occluded_diff}",
                ),
            ]
        )
        composite = np.vstack((top, bottom))
        filename = f"{gallery_index:03d}_{'pos' if row['has_stop_line'] else 'neg'}.png"
        save_png(images / filename, composite)
        record = {
            "index": gallery_index,
            "id": row["id"],
            "source": row["source"],
            "group": row.get("group", ""),
            "has_stop_line": row["has_stop_line"],
            "image_sha256": row["image_sha256"],
            "clean_mask_difference_pixels": dict(zip(("left", "right", "stop_line"), clean_diff)),
            "occluded_mask_difference_pixels": dict(
                zip(("left", "right", "stop_line"), occluded_diff)
            ),
            "render": f"images/{filename}",
        }
        records.append(record)
        cards.append(
            "<article><h2>{:03d} · {} · {}</h2><p>{} / {}</p>"
            '<a href="images/{}"><img loading="lazy" src="images/{}"></a></article>'.format(
                gallery_index,
                "stop-positive" if row["has_stop_line"] else "stop-negative",
                html.escape(row["id"]),
                html.escape(row["source"]),
                html.escape(row.get("group", "")),
                filename,
                filename,
            )
        )
        if (gallery_index + 1) % 10 == 0 or gallery_index + 1 == args.count:
            print(f"Rendered gallery images: {gallery_index + 1}/{args.count}", flush=True)

    audit = {
        "schema_version": 1,
        "selection": "50/50 stop-positive/negative, proportionally stratified by source/group",
        "count": len(records),
        "stop_positive": sum(record["has_stop_line"] for record in records),
        "stop_negative": sum(not record["has_stop_line"] for record in records),
        "by_source": dict(sorted(Counter(record["source"] for record in records).items())),
        "by_group": dict(sorted(Counter(record["group"] for record in records).items())),
        "validation_manifest": str(validation_path),
        "validation_manifest_sha256": sha256_file(validation_path),
        "reference_onnx_sha256": sha256_file(reference_path),
        "candidate_onnx_sha256": sha256_file(candidate_path),
        "panels": [
            "clean input",
            "ground truth",
            "FP32 clean",
            "INT8 clean",
            "occluded input",
            "ground truth on occluded input",
            "FP32 occluded",
            "INT8 occluded",
        ],
        "colors_bgr": {"left": [255, 255, 0], "right": [0, 165, 255], "stop_line": [255, 0, 255]},
        "samples": records,
    }
    write_json(output / "index.json", audit)
    page = """<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>INT8 PTQ visual comparison</title><style>
body{font-family:system-ui;background:#111722;color:#eef;margin:24px}article{margin:28px 0;padding:16px;background:#1b2433;border-radius:10px}
img{width:min(100%,1200px);image-rendering:pixelated}h1,h2{margin:.25em 0}p{color:#b9c4d6}</style>
<h1>Stop-line FP32 vs INT8 PTQ · 100 validation samples</h1>
<p>50 stop-positive + 50 stop-negative. Left=cyan, right=orange, stop line=magenta. Each image shows clean and deterministic occluded inputs.</p>
"""
    (output / "gallery.html").write_text(page + "".join(cards) + "</html>\n", encoding="utf-8")
    print(f"Saved {output / 'gallery.html'}")


if __name__ == "__main__":
    main()
