"""Serialized LaneResult that is decoded only when its contents are needed."""
import struct


class RawLane:
    """Routing needs only the capture stamp; forwarding reuses the same bytes.

    Any other field access (target-path validation, IR length exit) decodes the
    whole message once. `raw` is always the unmodified detector output.
    """

    def __init__(self, data, decode):
        self.raw = bytes(data)
        self._decode = decode
        self._message = None
        # Classic CDR: 4-byte encapsulation, then std_msgs/Header.stamp.
        if len(self.raw) >= 12 and self.raw[0] == 0 and self.raw[1] in (0, 1):
            sec, nanosec = struct.unpack_from('<iI' if self.raw[1] else '>iI', self.raw, 4)
        else:
            # Unknown encapsulation: never guess offsets, decode normally.
            stamp = self._decoded().header.stamp
            sec, nanosec = stamp.sec, stamp.nanosec
        self.capture_ns = sec * 1_000_000_000 + nanosec

    def _decoded(self):
        if self._message is None:
            self._message = self._decode(self.raw)
        return self._message

    def __getattr__(self, name):
        if name.startswith('_'):
            raise AttributeError(name)
        return getattr(self._decoded(), name)
