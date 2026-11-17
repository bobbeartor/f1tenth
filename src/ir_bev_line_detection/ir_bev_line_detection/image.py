"""ROS-independent, lossless adapter for the existing BGR-only lane detector."""
import numpy as np


def mono_to_bgr(data, width, height, step):
    if (width, height) != (120, 300):
        raise ValueError('Expected IR BEV width=120, height=300')
    if step < width or len(data) < step * height:
        raise ValueError('Invalid mono8 BEV stride or buffer length')
    gray = np.frombuffer(data, np.uint8, count=step*height).reshape(height, step)[:, :width]
    # No color conversion, resizing, or intensity remapping. Training loads
    # grayscale BEV PNGs as three identical channels with float /255 scaling.
    return np.repeat(gray[:, :, None], 3, axis=2).tobytes()
