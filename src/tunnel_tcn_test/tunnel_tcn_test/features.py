"""RGB point-event v1 features; source-time history and 10 Hz causal windows."""
from bisect import bisect_right, bisect_left
from collections import deque
import cv2
import numpy as np

FEATURES = (
    'mean', 'std', 'center_mean', 'surround_mean', 'p10', 'p90',
    'surround_center_contrast', 'center_relative_dark_fraction', 'spatial_gap_over_std',
    'mean_log_ratio_0p5s', 'center_log_ratio_0p5s', 'contrast_rate_0p5s',
    'dark_area_rate_0p5s', 'mean_prior_robust_z', 'center_prior_robust_z',
    'dark_area_prior_delta',
)

class FeatureExtractor:
    def __init__(self):
        self.rows, self.times = [], []

    def push(self, gray, timestamp):
        g = gray.astype(np.float32)
        h, w = g.shape; ya, yb = h//3, 2*h//3; xa, xb = w//3, 2*w//3
        center = g[ya:yb, xa:xb]
        surround = np.concatenate((g[:ya, xa:xb].ravel(), g[ya:yb, :xa].ravel(), g[ya:yb, xb:].ravel()))
        mean, std, cm, sm = float(g.mean()), float(g.std()), float(center.mean()), float(surround.mean())
        q25, med, q75 = np.percentile(surround, [25, 50, 75])
        dark = float(np.mean(center < med-0.5*max(q75-q25, 2.)))
        contrast = (sm-cm)/(sm+cm+1.)
        values = [mean/255, std/255, cm/255, sm/255,
                  *list(np.percentile(g, [10, 90])/255), contrast, dark,
                  float(np.clip((sm-cm)/max(std, 2.), -8, 8))]
        n = len(self.rows)
        lag = max(0, bisect_right(self.times, timestamp - .5 + 1e-7)-1)
        previous = self.rows[lag] if n else None
        dt = timestamp-self.times[lag] if n else 1.
        history = self.rows[bisect_left(self.times, timestamp-1.-1e-7):]
        if previous:
            logs = [np.log((mean+1)/(previous['mean']+1)), np.log((cm+1)/(previous['center']+1))]
            rates = [(contrast-previous['contrast'])/dt, (dark-previous['dark'])/dt]
        else:
            logs, rates = [0., 0.], [0., 0.]
        def robust_z(key, now):
            if not history: return 0.
            v = np.asarray([r[key] for r in history]); mid = np.median(v)
            return float(np.clip((now-mid)/max(1.4826*np.median(np.abs(v-mid)), 2.), -8, 8))
        values += logs+rates+[robust_z('mean', mean), robust_z('center', cm),
                               dark-float(np.median([r['dark'] for r in history])) if history else 0.]
        self.rows.append(dict(mean=mean, center=cm, contrast=contrast, dark=dark))
        self.times.append(timestamp)
        # Retain past 1 second plus one predecessor for causal lag lookup.
        while len(self.times) > 2 and self.times[1] < timestamp-1.1:
            self.times.pop(0)
            self.rows.pop(0)
        return np.asarray(values, np.float32)



def preprocess(image):
    if image.dtype != np.uint8 or image.ndim not in (2, 3):
        raise ValueError('Expected uint8 grayscale or RGB image')
    image = cv2.resize(image, (640, 400), interpolation=cv2.INTER_AREA)
    if image.ndim == 3:
        if image.shape[2] != 3:
            raise ValueError('Expected RGB channels')
        image = np.rint(image @ np.array([.299, .587, .114], np.float32)).clip(0, 255).astype(np.uint8)
    return image


class FeatureStream:
    def __init__(self, warmup_seconds=2.5, max_gap_seconds=.5, mean=None, std=None):
        self.warmup_seconds = float(warmup_seconds)
        self.max_gap_seconds = float(max_gap_seconds)
        self.mean = np.zeros(16, np.float32) if mean is None else np.asarray(mean, np.float32)
        self.std = np.ones(16, np.float32) if std is None else np.asarray(std, np.float32)
        if (self.mean.shape != (16,) or self.std.shape != (16,) or
                not np.isfinite(self.mean).all() or not np.isfinite(self.std).all() or (self.std <= 0).any()):
            raise ValueError('Invalid checkpoint scaler')
        if self.warmup_seconds < 0 or self.max_gap_seconds <= 0:
            raise ValueError('Invalid stream timing')
        self.reset()

    def reset(self):
        self.samples = deque()
        self.extractor = FeatureExtractor()
        self.started = self.last_time = None
        self.ready = self.discontinuity = False

    def _time(self, timestamp):
        if not np.isfinite(timestamp):
            raise ValueError('Invalid timestamp')
        gap = self.last_time is not None and (timestamp <= self.last_time or timestamp-self.last_time > self.max_gap_seconds)
        if gap:
            self.reset()
        self.discontinuity = gap
        if self.started is None:
            self.started = timestamp

    def push(self, image, timestamp):
        self._time(timestamp)
        feature = self.extractor.push(preprocess(image), timestamp)
        return self._context(feature, timestamp)

    def push_features(self, feature, timestamp):
        """Offline replay of frozen, unnormalized training features."""
        self._time(timestamp)
        return self._context(feature, timestamp)

    def _context(self, feature, timestamp):
        feature = np.asarray(feature, np.float32)
        if feature.shape != (16,) or not np.isfinite(feature).all():
            raise ValueError('Expected 16 finite features')
        self.samples.append((timestamp, (feature-self.mean)/self.std))
        while len(self.samples) > 1 and self.samples[1][0] < timestamp-3.2:
            self.samples.popleft()
        self.last_time = timestamp
        self.ready = timestamp-self.started >= self.warmup_seconds-1e-7
        return self._context_at(timestamp)

    def _context_at(self, timestamp):
        times = [t for t, _ in self.samples]
        targets = timestamp-np.arange(31, -1, -1)/10.
        indices = [max(0, bisect_right(times, t+1e-7)-1) for t in targets]
        x = np.stack([self.samples[i][1] for i in indices]).T
        mask = (targets >= self.started-1e-8).astype(np.float32)[None]
        return np.concatenate((x, mask), axis=0)[None].astype(np.float32)



class EventDecoder:
    def __init__(self, threshold=.5, cooldown=1., initial_state='unknown'):
        if initial_state not in ('unknown', 'inside', 'outside'):
            raise ValueError('Invalid initial_state')
        self.threshold, self.cooldown, self.initial_state = threshold, cooldown, initial_state
        self.reset()

    def reset(self):
        self.state = self.initial_state
        self.pause()

    def pause(self):
        self.active = np.zeros(2, bool)
        self.last = np.full(2, -np.inf)

    def step(self, scores, timestamp):
        p = np.asarray(scores)
        if p.shape != (2,) or not np.isfinite(p).all():
            raise ValueError('Expected finite scores [entry, exit]')
        active = p >= self.threshold
        candidates = np.flatnonzero(active & ~self.active & (timestamp-self.last >= self.cooldown))
        self.active = active
        if self.state != 'unknown':
            candidates = candidates[candidates != (0 if self.state == 'inside' else 1)]
        if not len(candidates):
            return None
        k = int(candidates[np.argmax(p[candidates])])
        self.state = 'inside' if k == 0 else 'outside'
        self.last[k] = timestamp
        return {'type': 'entry' if k == 0 else 'exit', 'score': float(p[k]), 'time_s': timestamp}


def sigmoid(logits):
    x = np.asarray(logits, np.float32).reshape(2)
    return np.exp(-np.logaddexp(0., -x))
