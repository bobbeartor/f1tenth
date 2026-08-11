#!/usr/bin/env python3
"""흰색 차선 검출 실험 도구 (독립 실행, ROS 무관).

사진을 넣으면 흰색 차선만 남긴 마스크를 만든다.

기본 사용:
    python3 white_line.py                     # input/ 폴더 전부 처리 -> result/
    python3 white_line.py --input 사진.png     # 파일 하나만

파라미터 실험:
    python3 white_line.py --gui               # 슬라이더로 실시간 조정 (추천)
    python3 white_line.py --threshold 45      # 값 하나만 바꿔서 일괄 처리
    python3 white_line.py --sweep             # 임계값 표 뽑기

결과 파일:
    {이름}_mask.png     최종 마스크 (흰색=차선)
    {이름}_overlay.jpg  원본 위 초록 표시  <- 합격 판정은 이것만 보면 됨
    {이름}_stages.jpg   단계별 분해        <- 어디서 새는지 확인용
"""

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    raise SystemExit("OpenCV 필요: python3 -m pip install opencv-python")

BASE_DIR = Path(__file__).resolve().parent
EXTS = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


# ---------------------------------------------------------------- 파라미터
class Params:
    """모든 튜닝 값. 슬라이더와 명령행 인자가 이 값을 바꾼다."""

    def __init__(self):
        self.work_width = 960      # 내부 처리 폭 (작을수록 빠름, 960 권장)
        self.blur = 5              # 블러 커널 (바닥 얼룩 완화)

        # 1단계: 화이트 탑햇 — 가는 밝은 선만 남기고 넓은 밝은 면은 제거
        self.tophat_kernel = 31    # 차선 폭보다 크게
        self.threshold = 60        # 탑햇 이진화 임계 (낮추면 더 많이 잡음)

        # 2단계: 어두움 문맥 — 차선은 검은 매트 위에만 있다
        self.dark_gate = 1         # 0이면 이 단계 끔
        self.dark_level = 70       # 이 값보다 어두우면 "검은 매트"
        self.dark_ratio = 12       # 주변에 검은 매트가 이 % 이상이어야 통과
        self.dark_window = 25      # 주변을 보는 창 크기

        # 3단계: 형태 필터 — 노이즈와 면 덩어리 제거
        self.min_area = 200        # 이보다 작은 조각은 버림
        self.blob_area = 1200      # 이보다 크고 뚱뚱하면 버림
        self.blob_aspect = 25      # 가로세로비 (x0.1). 25 = 2.5
        self.blob_fill = 45        # 채움율 %
        self.sliver = 1            # 상단 얇은 가로줄 제거 (원거리 바닥 이음새)
        self.sliver_height = 12
        self.sliver_top = 35       # 상단 몇 %까지 적용

        # 4단계: 기하 컷 — 지면이 아닌 영역을 통째로 제거
        self.horizon = 31          # 화면 위 몇 %를 잘라낼지 (수평선 컷)
        self.side_cut = 0          # 좌우 각각 몇 %를 잘라낼지 (0=안 함)
        self.bottom_cut = 16.67    # 화면 아래 1/6 제거 (바퀴/범퍼 영역)

        # 양쪽 어두움 조건: 차선은 양옆이 검은 매트, 매트 가장자리는 한쪽만
        self.bilateral = 0         # 1이면 양방향 조건 사용
        self.bilateral_span = 25   # 양옆을 얼마나 멀리까지 볼지 (px)

    def copy(self):
        p = Params()
        p.__dict__.update(self.__dict__)
        return p


def horizon_ratio(cut_beyond_m, fy=561.14, cy=352.62, cam_h=0.17,
                  pitch_deg=13.0, img_h=720):
    """전방 cut_beyond_m 미터에 해당하는 화면 행을 비율(%)로 반환.

    lane_mask.yaml의 카메라 보정 기본값을 그대로 사용한다.
    v(d) = cy + fy * tan( atan(h/d) - pitch )
    cut_beyond_m <= 0 이면 수평선(무한대) 기준.
    """
    import math
    pitch = math.radians(pitch_deg)
    if cut_beyond_m and cut_beyond_m > 0:
        alpha = math.atan(cam_h / cut_beyond_m) - pitch
    else:
        alpha = -pitch                      # d -> 무한대 = 수평선
    row = cy + fy * math.tan(alpha)
    return max(0.0, min(95.0, 100.0 * row / img_h))


def _odd(v, lo):
    v = max(int(round(v)), lo)
    return v if v % 2 else v + 1


# ---------------------------------------------------------------- 핵심 로직
def detect(bgr, p):
    """흰색 차선 검출. (mask, stages, info) 반환."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr

    # 처리 해상도로 축소
    shrink = 0 < p.work_width < gray.shape[1]
    if shrink:
        s = p.work_width / gray.shape[1]
        work = cv2.resize(gray, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    else:
        work = gray

    # 해상도가 바뀌어도 튜닝값이 유지되도록 크기 관련 값은 자동 보정
    r = work.shape[1] / 960.0
    k_top = _odd(p.tophat_kernel * r, 3)
    k_dark = _odd(p.dark_window * r, 3)
    a_min = max(10, int(round(p.min_area * r * r)))
    a_blob = max(50, int(round(p.blob_area * r * r)))
    h_sliver = max(2, int(round(p.sliver_height * r)))

    blurred = cv2.GaussianBlur(work, (_odd(p.blur, 1),) * 2, 0) \
        if p.blur > 1 else work

    # 1단계: 화이트 탑햇
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k_top, k_top))
    tophat = cv2.morphologyEx(blurred, cv2.MORPH_TOPHAT, kernel)
    _, cand = cv2.threshold(tophat, p.threshold, 255, cv2.THRESH_BINARY)
    stage_tophat = cand.copy()

    # 2단계: 어두움 문맥 게이트
    gate = np.full_like(cand, 255)
    if p.dark_gate:
        _, dark255 = cv2.threshold(blurred, p.dark_level, 255,
                                   cv2.THRESH_BINARY_INV)
        if p.bilateral:
            # 차선 테이프는 양옆(세로선) 또는 위아래(가로선)가 모두
            # 검은 매트다. 매트 가장자리는 한쪽만 어둡고 반대쪽은 밝은
            # 바닥이므로 여기서 걸러진다.
            # anchor 를 커널 한쪽 끝에 두면 그 반대 방향으로만 퍼진다.
            span = max(3, int(round(p.bilateral_span * r)))
            kh = np.ones((1, span), np.uint8)
            kv = np.ones((span, 1), np.uint8)
            left = cv2.dilate(dark255, kh, anchor=(span - 1, 0))
            right = cv2.dilate(dark255, kh, anchor=(0, 0))
            up = cv2.dilate(dark255, kv, anchor=(0, span - 1))
            down = cv2.dilate(dark255, kv, anchor=(0, 0))
            gate = cv2.bitwise_or(cv2.bitwise_and(left, right),
                                  cv2.bitwise_and(up, down))
        else:
            ratio = cv2.boxFilter((dark255 > 0).astype(np.float32), -1,
                                  (k_dark, k_dark))
            _, gate = cv2.threshold(ratio, p.dark_ratio / 100.0, 255,
                                    cv2.THRESH_BINARY)
            gate = gate.astype(np.uint8)
        cand = cv2.bitwise_and(cand, gate)
    # 기하 컷: 지면이 아닌 영역을 형태 분석 전에 제거
    rows_c, cols_c = cand.shape
    if p.horizon > 0:
        cand[: int(rows_c * p.horizon / 100.0), :] = 0
    if p.bottom_cut > 0:
        cand[rows_c - int(rows_c * p.bottom_cut / 100.0):, :] = 0
    if p.side_cut > 0:
        side = int(cols_c * p.side_cut / 100.0)
        cand[:, :side] = 0
        cand[:, cols_c - side:] = 0
    stage_gated = cand.copy()

    # 3단계: 형태 필터
    n, labels, stats, _ = cv2.connectedComponentsWithStats(cand, 8, cv2.CV_32S)
    out = np.zeros_like(cand)
    rows = cand.shape[0]
    kept = 0
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if area < a_min:
            continue
        aspect = max(bw, bh) / max(1, min(bw, bh))
        fill = area / max(1, bw * bh)
        if (area > a_blob and aspect < p.blob_aspect / 10.0
                and fill > p.blob_fill / 100.0):
            continue
        if (p.sliver and bw > 3 * bh and bh <= h_sliver
                and y < rows * p.sliver_top / 100.0):
            continue
        out[labels == i] = 255
        kept += 1

    if shrink:
        up = (gray.shape[1], gray.shape[0])
        f = cv2.INTER_NEAREST
        out = cv2.resize(out, up, interpolation=f)
        stage_tophat = cv2.resize(stage_tophat, up, interpolation=f)
        gate = cv2.resize(gate, up, interpolation=f)
        stage_gated = cv2.resize(stage_gated, up, interpolation=f)

    info = dict(components=kept, pixels=int((out > 0).sum()),
                percent=round(100.0 * (out > 0).mean(), 2))
    return out, (stage_tophat, gate, stage_gated), info


def overlay(bgr, mask):
    tinted = bgr.copy()
    tinted[mask > 0] = (0, 255, 0)
    return cv2.addWeighted(bgr, 0.5, tinted, 0.5, 0)


def _tag(im, text):
    im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR) if im.ndim == 2 else im.copy()
    cv2.rectangle(im, (0, 0), (im.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(im, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 255), 2)
    return im


def stage_sheet(bgr, stages, mask):
    h, w = bgr.shape[:2]
    sz = (430, max(1, int(430 * h / w)))
    tophat, gate, gated = stages
    top = np.hstack([_tag(cv2.resize(bgr, sz), "1.input"),
                     _tag(cv2.resize(tophat, sz), "2.tophat"),
                     _tag(cv2.resize(gate, sz), "3.dark_gate")])
    bot = np.hstack([_tag(cv2.resize(gated, sz), "4.AND"),
                     _tag(cv2.resize(mask, sz), "5.mask"),
                     _tag(cv2.resize(overlay(bgr, mask), sz), "6.overlay")])
    return np.vstack([top, bot])


# ---------------------------------------------------------------- 입출력
def imread(path):
    data = np.fromfile(str(path), dtype=np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"읽을 수 없음: {path}")
    return img


def imwrite(path, img):
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(path.suffix, img)
    if not ok:
        raise ValueError(f"저장 실패: {path}")
    buf.tofile(str(path))


def collect(path):
    if path.is_file():
        return [path]
    return sorted(p for p in path.rglob("*")
                  if p.suffix.lower() in EXTS and p.is_file())


# ---------------------------------------------------------------- 슬라이더
SLIDERS = [
    ("threshold", 255), ("tophat_kernel", 81), ("blur", 15),
    ("dark_gate", 1), ("dark_level", 255), ("dark_ratio", 100),
    ("dark_window", 81), ("min_area", 3000), ("blob_area", 8000),
    ("blob_aspect", 100), ("blob_fill", 100),
    ("sliver", 1), ("sliver_height", 60), ("sliver_top", 100),
    ("horizon", 90), ("side_cut", 40), ("bottom_cut", 40),
    ("bilateral", 1), ("bilateral_span", 80),
]


def run_gui(images, p):
    """슬라이더로 실시간 조정. n/p=사진 이동, s=현재 값 저장, q=종료."""
    win = "white line tuner"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 760)
    ctrl = "params"
    cv2.namedWindow(ctrl, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(ctrl, 420, 560)
    for name, top in SLIDERS:
        cv2.createTrackbar(name, ctrl, getattr(p, name), top, lambda _v: None)

    index = 0
    cache = {}
    print("슬라이더로 조정 | n=다음 사진  p=이전  s=값 저장  q=종료")
    while True:
        for name, _ in SLIDERS:
            setattr(p, name, cv2.getTrackbarPos(name, ctrl))
        path = images[index]
        if path not in cache:
            cache[path] = cv2.resize(imread(path), (1280, 720))
        bgr = cache[path]
        mask, stages, info = detect(bgr, p)
        view = np.hstack([overlay(bgr, mask),
                          cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        view = cv2.resize(view, (1280, 360))
        cv2.putText(view, f"{path.name}  comp={info['components']}  "
                          f"{info['percent']}%",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow(win, view)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord('n'):
            index = (index + 1) % len(images)
        if key == ord('p'):
            index = (index - 1) % len(images)
        if key == ord('s'):
            print("\n현재 파라미터 (명령행에 그대로 붙여넣기):")
            print("  " + " ".join(
                f"--{n.replace('_', '-')} {getattr(p, n)}"
                for n, _ in SLIDERS))
            print()
    cv2.destroyAllWindows()


# ---------------------------------------------------------------- main
def main():
    p = Params()
    ap = argparse.ArgumentParser(description="흰색 차선 검출 실험 도구")
    ap.add_argument("--input", type=Path, default=BASE_DIR / "input")
    ap.add_argument("--result", type=Path, default=BASE_DIR / "result")
    ap.add_argument("--gui", action="store_true", help="슬라이더 실시간 조정")
    ap.add_argument("--sweep", action="store_true", help="임계값 표 출력")
    ap.add_argument("--work-width", type=int, default=p.work_width)
    ap.add_argument("--cut-beyond", type=float, default=None,
                    help="전방 N미터 너머를 잘라냄 (카메라 기하로 자동 계산). "
                         "0이면 수평선 기준. 지정하면 --horizon 을 덮어씀")
    for name, _ in SLIDERS:
        ap.add_argument(f"--{name.replace('_', '-')}", type=int,
                        default=getattr(p, name))
    args = ap.parse_args()
    for name, _ in SLIDERS:
        setattr(p, name, getattr(args, name))
    p.work_width = args.work_width
    if args.cut_beyond is not None:
        p.horizon = int(round(horizon_ratio(args.cut_beyond)))
        print(f"기하 계산: 전방 {args.cut_beyond} m 너머 컷 "
              f"-> 화면 위 {p.horizon}% 제거")

    images = collect(args.input)
    if not images:
        print(f"입력 이미지가 없습니다: {args.input}")
        print("사진을 input/ 폴더에 넣거나 --input 으로 경로를 지정하세요.")
        sys.exit(1)

    if args.gui:
        run_gui(images, p)
        return

    if args.sweep:
        print(f"{'kernel':>7} {'thresh':>7} | {'성분':>5} {'차선비율%':>9}")
        for k in (21, 31, 41, 51):
            for th in (35, 45, 55, 65, 75):
                q = p.copy()
                q.tophat_kernel, q.threshold = k, th
                comp = pct = 0.0
                for path in images:
                    _, _, info = detect(imread(path), q)
                    comp += info["components"]
                    pct += info["percent"]
                print(f"{k:>7} {th:>7} | {comp:>5.0f} {pct / len(images):>9.2f}")
        return

    print(f"이미지 {len(images)}장 처리 (work_width={p.work_width})")
    for path in images:
        bgr = imread(path)
        mask, stages, info = detect(bgr, p)
        stem = path.stem
        imwrite(args.result / f"{stem}_mask.png", mask)
        imwrite(args.result / f"{stem}_overlay.jpg", overlay(bgr, mask))
        imwrite(args.result / f"{stem}_stages.jpg",
                stage_sheet(bgr, stages, mask))
        print(f"  {path.name}: 성분 {info['components']}개, "
              f"차선 {info['pixels']}px ({info['percent']}%)")
    print(f"완료 -> {args.result}")


if __name__ == "__main__":
    main()
