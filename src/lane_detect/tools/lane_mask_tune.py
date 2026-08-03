#!/usr/bin/env python3
"""lane_mask 오프라인 튜닝 도구.

C++ 노드(src/lane_mask_node.cpp)와 완전히 동일한 4단 알고리즘을 돌린다.
여기서 찾은 값을 config/lane_mask.yaml에 그대로 옮기면 실차 동작이 같다.

사용법:
    python3 lane_mask_tune.py                        # input/ -> result/
    python3 lane_mask_tune.py --tophat-threshold 45  # 파라미터 실험
    python3 lane_mask_tune.py --sweep                # 임계값 자동 스윕표

각 이미지마다 저장하는 것:
    {이름}_mask.png     최종 마스크 (255=차선)
    {이름}_overlay.jpg  원본 위 초록 표시 (합격 판정용)
    {이름}_stages.jpg   단계별 분해 (어디서 새는지 확인용)
"""

import argparse
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    raise SystemExit("OpenCV 필요: python3 -m pip install opencv-python")

BASE_DIR = Path(__file__).resolve().parent
IMAGE_EXTENSIONS = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


def make_odd(value, minimum):
    value = max(int(round(value)), minimum)
    return value if value % 2 == 1 else value + 1


def compute_mask(luma, a):
    """C++ Impl::computeMask()와 1:1 대응."""
    downscale = a.process_width > 0 and a.process_width < luma.shape[1]
    if downscale:
        scale = a.process_width / luma.shape[1]
        work = cv2.resize(luma, None, fx=scale, fy=scale,
                          interpolation=cv2.INTER_AREA)
    else:
        work = luma

    # 해상도 자동 스케일: 길이는 비례, 면적은 제곱 비례
    r = work.shape[1] / a.param_reference_width
    tophat_k = make_odd(a.tophat_kernel * r, 3)
    dark_win = make_odd(a.dark_window * r, 3)
    min_area = max(10, int(round(a.min_area * r * r)))
    blob_area = max(50, int(round(a.blob_area * r * r)))
    sliver_h = max(2, int(round(a.sliver_max_height * r)))
    bilateral_span = max(3, int(round(a.bilateral_span_px * r)))

    blurred = (cv2.GaussianBlur(work, (a.blur_kernel, a.blur_kernel), 0)
               if a.blur_kernel > 1 else work)

    # ① 화이트 탑햇: 넓은 밝은 면 제거, 가는 밝은 선만 남김
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (tophat_k, tophat_k))
    tophat = cv2.morphologyEx(blurred, cv2.MORPH_TOPHAT, kernel)
    _, candidate = cv2.threshold(tophat, a.tophat_threshold, 255,
                                 cv2.THRESH_BINARY)
    stage_tophat = candidate.copy()

    # ② 어두움-문맥 게이트: 차선 테이프는 검은 매트 위에만 있다
    gate = np.full_like(candidate, 255)
    if a.bilateral_dark_enabled:
        _, dark = cv2.threshold(blurred, a.dark_threshold, 1,
                                cv2.THRESH_BINARY_INV)
        kh = cv2.getStructuringElement(
            cv2.MORPH_RECT, (bilateral_span, 1))
        kv = cv2.getStructuringElement(
            cv2.MORPH_RECT, (1, bilateral_span))
        left = cv2.dilate(
            dark, kh, anchor=(bilateral_span - 1, 0),
            borderType=cv2.BORDER_CONSTANT, borderValue=0)
        right = cv2.dilate(
            dark, kh, anchor=(0, 0),
            borderType=cv2.BORDER_CONSTANT, borderValue=0)
        up = cv2.dilate(
            dark, kv, anchor=(0, bilateral_span - 1),
            borderType=cv2.BORDER_CONSTANT, borderValue=0)
        down = cv2.dilate(
            dark, kv, anchor=(0, 0),
            borderType=cv2.BORDER_CONSTANT, borderValue=0)
        vertical_stroke = cv2.bitwise_and(left, right)
        horizontal_stroke = cv2.bitwise_and(up, down)
        gate = cv2.bitwise_or(vertical_stroke, horizontal_stroke) * 255
        candidate = cv2.bitwise_and(candidate, gate)
    elif a.dark_gate_enabled:
        _, dark = cv2.threshold(blurred, a.dark_threshold, 1,
                                cv2.THRESH_BINARY_INV)
        ratio_map = cv2.boxFilter(dark.astype(np.float32), -1,
                                  (dark_win, dark_win))
        _, gate = cv2.threshold(ratio_map, a.dark_ratio, 255,
                                cv2.THRESH_BINARY)
        gate = gate.astype(np.uint8)
        candidate = cv2.bitwise_and(candidate, gate)

    alpha = (np.arctan(a.camera_height_m / a.cut_beyond_m)
             - np.deg2rad(a.camera_pitch_deg)
             if a.cut_beyond_m > 0.0
             else -np.deg2rad(a.camera_pitch_deg))
    row_ref = a.camera_cy + a.camera_fy * np.tan(alpha)
    cut_row = int(round(
        row_ref / a.reference_image_height * candidate.shape[0]))
    cut_row = max(0, min(cut_row, max(0, candidate.shape[0] - 1)))
    if cut_row > 0:
        candidate[:cut_row, :] = 0

    if a.bottom_cut_ratio > 0.0:
        bottom_rows = min(
            candidate.shape[0],
            int(round(candidate.shape[0] * a.bottom_cut_ratio)),
        )
        if bottom_rows > 0:
            candidate[-bottom_rows:, :] = 0
    stage_gated = candidate.copy()

    # ③ 형태 필터
    n, labels, stats, _ = cv2.connectedComponentsWithStats(
        candidate, 8, cv2.CV_32S)
    filtered = np.zeros_like(candidate)
    rows = candidate.shape[0]
    kept = 0
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if area < min_area:
            continue
        aspect = max(bw, bh) / max(1, min(bw, bh))
        fill = area / max(1, bw * bh)
        if area > blob_area and aspect < a.blob_aspect and fill > a.blob_fill:
            continue
        if (a.sliver_filter_enabled and bw > 3 * bh and bh <= sliver_h
                and y < rows * a.sliver_top_ratio):
            continue
        filtered[labels == i] = 255
        kept += 1

    if downscale:
        full = cv2.resize(filtered, (luma.shape[1], luma.shape[0]),
                          interpolation=cv2.INTER_NEAREST)
    else:
        full = filtered
    scaled = dict(
        tophat_k=tophat_k,
        dark_win=dark_win,
        bilateral_span=bilateral_span,
        cut_row=cut_row,
        min_area=min_area,
        blob_area=blob_area,
        sliver_h=sliver_h,
    )
    return full, kept, (stage_tophat, gate, stage_gated, filtered), scaled


def label(im, text):
    im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR) if im.ndim == 2 else im.copy()
    cv2.rectangle(im, (0, 0), (im.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(im, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 255), 2)
    return im


def stage_grid(bgr, stages, mask, blend):
    h, w = bgr.shape[:2]
    sz = (440, int(440 * h / w))
    tophat, gate, gated, _ = stages
    row1 = np.hstack([label(cv2.resize(bgr, sz), "1.input"),
                      label(cv2.resize(tophat, sz), "2.tophat_bin"),
                      label(cv2.resize(gate, sz), "3.dark_gate")])
    row2 = np.hstack([label(cv2.resize(gated, sz), "4.AND"),
                      label(cv2.resize(mask, sz), "5.final_mask"),
                      label(cv2.resize(blend, sz), "6.overlay")])
    return np.vstack([row1, row2])


def read_image(path):
    data = np.fromfile(str(path), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"읽을 수 없음: {path}")
    return image


def write_image(path, image):
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(path.suffix, image)
    if not ok:
        raise ValueError(f"저장 실패: {path}")
    encoded.tofile(str(path))


def build_parser():
    p = argparse.ArgumentParser(description="lane_mask 오프라인 튜닝")
    p.add_argument("--input", type=Path, default=BASE_DIR / "input")
    p.add_argument("--result", type=Path, default=BASE_DIR / "result")
    p.add_argument("--sweep", action="store_true",
                   help="임계값 스윕표만 출력하고 종료")
    # 아래 이름은 config/lane_mask.yaml의 키와 동일하다.
    p.add_argument("--process-width", type=int, default=960)
    p.add_argument("--param-reference-width", type=int, default=960)
    p.add_argument("--blur-kernel", type=int, default=5)
    p.add_argument("--tophat-kernel", type=int, default=31)
    p.add_argument("--tophat-threshold", type=int, default=45)
    p.add_argument("--dark-gate-enabled", type=int, default=1)
    p.add_argument("--dark-threshold", type=int, default=70)
    p.add_argument("--dark-ratio", type=float, default=0.20)
    p.add_argument("--dark-window", type=int, default=25)
    p.add_argument("--bilateral-dark-enabled", type=int, default=1)
    p.add_argument("--bilateral-span-px", type=int, default=25)
    p.add_argument("--min-area", type=int, default=400)
    p.add_argument("--blob-area", type=int, default=1200)
    p.add_argument("--blob-aspect", type=float, default=2.5)
    p.add_argument("--blob-fill", type=float, default=0.45)
    p.add_argument("--sliver-filter-enabled", type=int, default=1)
    p.add_argument("--sliver-max-height", type=int, default=12)
    p.add_argument("--sliver-top-ratio", type=float, default=0.35)
    p.add_argument("--cut-beyond-m", type=float, default=2.0)
    p.add_argument("--camera-height-m", type=float, default=0.17)
    p.add_argument("--camera-pitch-deg", type=float, default=13.0)
    p.add_argument("--camera-fy", type=float, default=561.136352539)
    p.add_argument("--camera-cy", type=float, default=352.621124268)
    p.add_argument("--reference-image-height", type=int, default=720)
    p.add_argument("--bottom-cut-ratio", type=float, default=0.15)
    return p


def main():
    args = build_parser().parse_args()
    args.dark_gate_enabled = bool(args.dark_gate_enabled)
    args.bilateral_dark_enabled = bool(args.bilateral_dark_enabled)
    args.sliver_filter_enabled = bool(args.sliver_filter_enabled)

    input_path = args.input
    images = ([input_path] if input_path.is_file() else
              sorted(p for p in input_path.rglob("*")
                     if p.suffix.lower() in IMAGE_EXTENSIONS and p.is_file()))
    if not images:
        raise SystemExit(f"입력 이미지 없음: {input_path}")

    if args.sweep:
        print(f"{'tophat_k':>9} {'thresh':>7} | {'성분':>5} {'좌픽셀':>8} "
              f"{'우픽셀':>8}   (파일별 합계)")
        for k in (21, 31, 41):
            for th in (35, 45, 55, 60, 70):
                total_kept = left = right = 0
                for path in images:
                    gray = cv2.cvtColor(read_image(path), cv2.COLOR_BGR2GRAY)
                    args.tophat_kernel, args.tophat_threshold = k, th
                    mask, kept, _, _ = compute_mask(gray, args)
                    half = mask.shape[1] // 2
                    total_kept += kept
                    left += int((mask[:, :half] > 0).sum())
                    right += int((mask[:, half:] > 0).sum())
                print(f"{k:>9} {th:>7} | {total_kept:>5} {left:>8} {right:>8}")
        return

    print(f"이미지 {len(images)}장 처리 (process_width={args.process_width})")
    for path in images:
        bgr = read_image(path)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        mask, kept, stages, scaled = compute_mask(gray, args)
        overlay = bgr.copy()
        overlay[mask > 0] = (0, 255, 0)
        blend = cv2.addWeighted(bgr, 0.55, overlay, 0.45, 0)
        stem = path.stem
        write_image(args.result / f"{stem}_mask.png", mask)
        write_image(args.result / f"{stem}_overlay.jpg", blend)
        write_image(args.result / f"{stem}_stages.jpg",
                    stage_grid(bgr, stages, mask, blend))
        half = mask.shape[1] // 2
        print(f"  {path.name}: 성분 {kept}개, "
              f"좌 {int((mask[:, :half] > 0).sum())}px / "
              f"우 {int((mask[:, half:] > 0).sum())}px  "
              f"[스케일 적용값 {scaled}]")
    print(f"완료 -> {args.result}")


if __name__ == "__main__":
    main()
