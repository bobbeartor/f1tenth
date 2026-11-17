"""Independent loading reporter; a heartbeat is not TensorRT progress proof."""
import threading
import time


class LoadingStatus:
    def __init__(self, publish, interval=1.):
        self.publish = publish
        self.started = self.stage_at = time.monotonic()
        self.status = ''
        self.sequence = 0
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.interval = interval
        self.thread.start()

    def send(self):
        if not self.status:
            return
        text = self.status
        now = time.monotonic()
        text += (f' | elapsed={now-self.started:.0f}s'
                 f' | stage_age={now-self.stage_at:.0f}s')
        if self.status.startswith('LOADING | '):
            self.sequence += 1
            text += f' | monitor_seq={self.sequence}'
        self.publish(text)

    def update(self, status):
        with self.lock:
            if status != self.status:
                self.stage_at = time.monotonic()
            self.status = status
            self.send()

    def stage(self, text):
        self.update('LOADING | ' + text)

    def run(self):
        while not self.stopping.wait(self.interval):
            with self.lock:
                if self.status.startswith('LOADING | '):
                    try:
                        self.send()
                    except Exception:
                        return  # No successful heartbeat must be inferred after publication fails.

    def close(self):
        self.stopping.set()
        self.thread.join()
