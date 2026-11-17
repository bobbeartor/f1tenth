"""Link-loss watchdog and latest-only client queue for the remote preview (no ROS)."""
import threading


class LinkWatchdog:
    """Fires once per heartbeat loss while the controller is STARTED.

    No heartbeat ever received -> never fires (driving without a client is unprotected).
    timeout_sec <= 0 disables the watchdog.
    """

    def __init__(self, timeout_sec):
        self.timeout_sec = float(timeout_sec)
        self.last = None
        self.fired = False
        self.lock = threading.Lock()

    def heartbeat(self, now):
        with self.lock:
            self.last = now
            self.fired = False

    def heartbeat_age_ms(self, now):
        with self.lock:
            return None if self.last is None else (now - self.last) * 1000.0

    def should_stop(self, now, started):
        with self.lock:
            if self.timeout_sec <= 0 or self.last is None or self.fired or not started:
                return False
            if now - self.last > self.timeout_sec:
                self.fired = True
                return True
            return False


class LatestSlot:
    """Single-item mailbox: put overwrites, get waits; a slow reader only ever sees the newest."""

    def __init__(self):
        self.cond = threading.Condition()
        self.item = None
        self.closed = False

    def put(self, item):
        with self.cond:
            self.item = item
            self.cond.notify()

    def get(self, timeout):
        with self.cond:
            if self.item is None and not self.closed:
                self.cond.wait(timeout)
            item, self.item = self.item, None
            return None if self.closed else item

    def close(self):
        with self.cond:
            self.closed = True
            self.cond.notify_all()
