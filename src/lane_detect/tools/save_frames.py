#!/usr/bin/env python3
"""camera_driver가 발행하는 영상을 사진으로 저장한다 (튜닝 데이터셋용).

camera_driver를 publish_enabled:=true로 띄운 뒤 이 스크립트를 실행한다.
OAK는 한 프로세스만 열 수 있으므로, 카메라를 직접 여는 대신 이미 돌고 있는
드라이버의 토픽을 구독한다. 저장되는 사진은 실제 파이프라인이 쓰는 것과
동일한 온디바이스 왜곡 보정 프레임이다.

사용법:
    python3 save_frames.py                 # 미리보기 창에서 s키로 저장
    python3 save_frames.py --auto 1.0      # 1초마다 자동 저장
    python3 save_frames.py --out ~/lane/input

키:
    s = 현재 프레임 저장
    q = 종료
"""

import argparse
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    import cv2
except ImportError:
    raise SystemExit("OpenCV 필요: python3 -m pip install opencv-python")


def to_bgr(msg):
    """NV12 또는 mono8 메시지를 BGR 이미지로."""
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if msg.encoding == "nv12":
        rows = h * 3 // 2
        planes = buf[: step * rows].reshape(rows, step)[:, :w]
        return cv2.cvtColor(planes, cv2.COLOR_YUV2BGR_NV12)

    if msg.encoding in ("mono8", "8UC1"):
        gray = buf[: step * h].reshape(h, step)[:, :w]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    if msg.encoding in ("bgr8", "rgb8"):
        img = buf[: step * h].reshape(h, step)[:, : w * 3].reshape(h, w, 3)
        return img if msg.encoding == "bgr8" else img[:, :, ::-1].copy()

    raise ValueError(f"지원하지 않는 encoding: {msg.encoding}")


class FrameSaver(Node):
    def __init__(self, args):
        super().__init__("frame_saver")
        self.out_dir = Path(args.out).expanduser()
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.auto = args.auto
        self.count = len(list(self.out_dir.glob("track_*.png")))
        self.last_auto = self.get_clock().now()
        self.subscription = self.create_subscription(
            Image, args.topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"구독 {args.topic} -> 저장 {self.out_dir}  (s=저장, q=종료)")

    def save(self, bgr):
        self.count += 1
        path = self.out_dir / f"track_{self.count:03d}.png"
        ok, buf = cv2.imencode(".png", bgr)
        if ok:
            buf.tofile(str(path))
            self.get_logger().info(f"저장: {path.name}  {bgr.shape[1]}x{bgr.shape[0]}")

    def on_image(self, msg):
        try:
            bgr = to_bgr(msg)
        except ValueError as error:
            self.get_logger().warn(str(error), throttle_duration_sec=5.0)
            return

        view = cv2.resize(bgr, (960, int(960 * bgr.shape[0] / bgr.shape[1])))
        cv2.putText(view, f"saved={self.count}  s=save  q=quit",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("camera", view)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            self.save(bgr)
        elif key == ord("q"):
            raise SystemExit(0)

        if self.auto > 0.0:
            now = self.get_clock().now()
            if (now - self.last_auto).nanoseconds / 1e9 >= self.auto:
                self.last_auto = now
                self.save(bgr)


def main():
    ap = argparse.ArgumentParser(description="camera_driver 프레임 저장")
    ap.add_argument("--topic", default="/camera/image_rect")
    ap.add_argument("--out", default="~/lane/input")
    ap.add_argument("--auto", type=float, default=0.0,
                    help="초 단위 자동 저장 간격 (0이면 수동만)")
    args = ap.parse_args()

    rclpy.init()
    node = FrameSaver(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
