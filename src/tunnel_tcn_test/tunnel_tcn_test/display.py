"""ROS-independent preview rendering and camera message decoding."""
import cv2
import numpy as np


# Limited-range BT.601 luma to the full-range brightness used by training.
_LUMA_TO_GRAY = np.rint(np.clip((np.arange(256, dtype=np.float32) - 16) * (255 / 219), 0, 255)).astype(np.uint8)


def mono_image(message):
    """Decode NV12 directly to 640x400 brightness; preserve external RGB/mono inputs."""
    h, w, step = message.height, message.width, message.step
    if h <= 0 or w <= 0:
        raise ValueError('Invalid image dimensions')
    encoding = message.encoding.lower()
    channels = 3 if encoding in ('rgb8', 'bgr8') else 1
    rows = h * 3 // 2 if encoding == 'nv12' else h
    if encoding not in ('nv12', 'rgb8', 'bgr8', 'mono8'):
        raise ValueError('Expected nv12, rgb8, bgr8 or mono8')
    if encoding == 'nv12' and (h % 2 or w % 2):
        raise ValueError('NV12 needs even dimensions')
    if step < w*channels or len(message.data) < step*rows:
        raise ValueError('Invalid image stride/buffer length')
    if encoding == 'nv12':
        # View only Y, respecting row padding. UV is validated but never processed.
        y = np.frombuffer(message.data, np.uint8, count=step*h).reshape(h, step)[:, :w]
        small = cv2.resize(y, (640, 400), interpolation=cv2.INTER_AREA)
        return cv2.LUT(small, _LUMA_TO_GRAY)
    raw = np.frombuffer(message.data, np.uint8, count=step*h).reshape(h, step)[:, :w*channels].copy()
    if channels == 3:
        image = raw.reshape(h, w, 3)
        return image[:, :, ::-1].copy() if encoding == 'bgr8' else image
    return raw


def footer(gray, probabilities, state, last_event, status, fps, infer_ms, total_ms, dropped, settings):
    resized = cv2.resize(gray, (960, 600), interpolation=cv2.INTER_AREA)
    picture = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR if resized.ndim == 2 else cv2.COLOR_RGB2BGR)
    canvas = cv2.copyMakeBorder(picture, 0, 160, 0, 0, cv2.BORDER_CONSTANT, value=(22, 24, 28))
    lines = [
        f'RGB FP32 | {status} | state: {state.upper()}',
        'ENTRY score %.3f   EXIT score %.3f' % tuple(probabilities),
        f'Last event: {last_event} | source FPS {fps:.1f} | missed frames ~{dropped}',
        f'TRT+transfer {infer_ms:.2f} ms | frame processing {total_ms:.2f} ms | R: new drive/reset  Q: quit',
        f'Camera: {settings}',
    ]
    for j, line in enumerate(lines):
        color = (100, 230, 120) if j == 0 and status == 'LIVE' else (225, 225, 225)
        cv2.putText(canvas, line, (14, 624 + 29 * j), cv2.FONT_HERSHEY_SIMPLEX, .54, color, 1, cv2.LINE_AA)
    return canvas
