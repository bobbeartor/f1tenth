"""Read-only Linux/Jetson device metrics, sampled away from the GUI/ROS loop."""
import math
from pathlib import Path
import shutil
import threading
import time


class SystemProbe:
    def __init__(self, proc='/proc', sys='/sys', disk='/'):
        self.proc, self.sys, self.disk = Path(proc), Path(sys), disk
        self.previous_cpu = None
        self.previous_cores = {}
        self.partial = {}
        self.errors = []

    @staticmethod
    def read(path):
        try:
            return path.read_text().strip()
        except (OSError, UnicodeError):
            return ''

    def number(self, path):
        try:
            value = float(self.read(path))
            return value if math.isfinite(value) else None
        except ValueError:
            return None

    def paths(self, pattern, root=None):
        found = []
        try:
            for path in (root or self.sys).glob(pattern):
                found.append(path)
        except (OSError, RuntimeError) as error:
            self.errors.append(f'{pattern}: {error}')
        return found

    def sample(self):
        result = {}
        self.partial = result
        self.errors = []
        # guest/guest_nice are already included in user/nice, so use first 8 fields.
        stat = self.read(self.proc / 'stat').splitlines()
        try:
            cpu = [int(v) for v in stat[0].split()[1:9]]
            if stat[0].split()[0] != 'cpu' or len(cpu) < 4 or min(cpu) < 0:
                raise ValueError('Invalid CPU counters')
            total, idle = sum(cpu), cpu[3] + (cpu[4] if len(cpu) > 4 else 0)
            if self.previous_cpu is not None:
                delta = total - self.previous_cpu[0]
                idle_delta = idle - self.previous_cpu[1]
                if delta > 0 and 0 <= idle_delta <= delta:
                    result['cpu_pct'] = 100 * (1 - idle_delta / delta)
            self.previous_cpu = total, idle
            result['cpu_cores'] = sum(line.split()[0][3:].isdigit() for line in stat if line)
        except (IndexError, ValueError):
            self.previous_cpu = None
        cores = {}
        current_cores = {}
        for line in stat:
            fields = line.split()
            if not fields or not fields[0].startswith('cpu') or not fields[0][3:].isdigit():
                continue
            try:
                values = [int(v) for v in fields[1:9]]
                total, idle = sum(values), values[3] + (values[4] if len(values) > 4 else 0)
                previous = self.previous_cores.get(fields[0])
                if previous and total > previous[0]:
                    usage = 100 * (1 - (idle - previous[1]) / (total - previous[0]))
                    if 0 <= usage <= 100:
                        cores[fields[0]] = usage
                current_cores[fields[0]] = (total, idle)
            except (ValueError, IndexError):
                continue
        self.previous_cores = current_cores
        if cores:
            result['cpu_core_pct'] = cores
        mem = {}
        for line in self.read(self.proc / 'meminfo').splitlines():
            try:
                key, value = line.split(':', 1)
                mem[key] = int(value.split()[0])
            except (ValueError, IndexError):
                continue
        for prefix, total_key, free_key in (
                ('ram', 'MemTotal', 'MemAvailable'), ('swap', 'SwapTotal', 'SwapFree')):
            total, free = mem.get(total_key), mem.get(free_key)
            if total is not None and free is not None and 0 <= free <= total:
                result[prefix + '_total_gib'] = total / 1024**2
                result[prefix + '_used_gib'] = (total - free) / 1024**2
        try:
            load = [float(v) for v in self.read(self.proc / 'loadavg').split()[:3]]
            if len(load) == 3 and all(math.isfinite(v) and v >= 0 for v in load):
                result['load'] = load
        except ValueError:
            pass
        cpu_freq = [self.number(p) for p in self.paths('devices/system/cpu/cpufreq/policy*/scaling_cur_freq')]
        cpu_freq = [v for v in cpu_freq if v is not None and v > 0]
        if cpu_freq:
            result['cpu_mhz'] = max(cpu_freq) / 1000
        # Jetson nvgpu load is per-mille (0..1000), not percent.
        gpu_paths = [self.sys / p for p in (
            'devices/gpu.0', 'devices/platform/gpu.0', 'devices/platform/17000000.gpu',
            'devices/platform/17000000.ga10b', 'devices/platform/bus@0/17000000.gpu',
            'devices/platform/host1x/gpu.0', 'devices/57000000.gpu')]
        devfreq = []
        for path in self.paths('class/devfreq/*'):
            try:
                identity = str(path.resolve()).lower()
            except (OSError, RuntimeError) as error:
                self.errors.append(f'{path}: {error}')
                identity = path.name.lower()
            if any(name in identity for name in ('gpu', 'ga10b', 'gv11b')):
                devfreq.append(path)
        gpu_paths += devfreq + [p / 'device' for p in devfreq]
        gpu_paths += self.paths('devices/platform/*gpu*') + self.paths('devices/platform/bus@0/*gpu*')
        for path in gpu_paths:
            value = self.number(path / 'load')
            if value is not None and 0 <= value <= 1000:
                result['gpu_pct'] = value / 10
                break
        for path in devfreq:
            value = self.number(path / 'cur_freq')
            if value is not None and value > 0:
                result['gpu_mhz'] = value / 1e6
                break
        temperatures = {'cpu': [], 'gpu': [], 'soc': []}
        for path in self.paths('class/thermal/thermal_zone*'):
            kind = self.read(path / 'type').lower()
            value = self.number(path / 'temp')
            if value is None or not -40000 <= value <= 150000:
                continue
            # Tmargin zones report thermal headroom, not actual temperature.
            if 'margin' in kind:
                continue
            for key in temperatures:
                if key in kind:
                    temperatures[key].append(value / 1000)
        for key, values in temperatures.items():
            if values:
                result[key + '_temp_c'] = max(values)
        try:
            result['disk_free_gib'] = shutil.disk_usage(self.disk).free / 1024**3
        except OSError:
            pass
        try:
            power = self.power_w()
            if power is not None:
                result['power_w'] = power
        except (OSError, RuntimeError, ValueError) as error:
            self.errors.append(f'power: {error}')
        if self.errors:
            result['error'] = '; '.join(self.errors)
        if 'cpu_pct' not in result and self.previous_cpu is None:
            result['error'] = result.get('error', '') + ' CPU counters unavailable at /proc/stat.'
        return result

    # Board input rail priority (ported from the removed C++ performance measurement).
    POWER_RAILS = {'VDD_IN': 0, '5V_IN': 1, 'POM_5V_IN': 2, 'VIN_SYS_5V0': 3}

    def power_w(self):
        candidates = []
        for device in self.paths('bus/i2c/drivers/ina3221*/*'):
            for label in self.paths('hwmon/hwmon*/in*_label', device):
                rank = self.POWER_RAILS.get(self.read(label))
                channel = label.name[2:-len('_label')]
                if rank is None or not channel.isdigit():
                    continue
                volt = self.number(label.parent / f'in{channel}_input')
                curr = self.number(label.parent / f'curr{channel}_input')
                if volt is not None and curr is not None:
                    candidates.append((rank, volt * curr / 1e6))  # mV * mA -> W
            for label in self.paths('iio:device*/rail_name_*', device):
                rank = self.POWER_RAILS.get(self.read(label))
                channel = label.name[len('rail_name_'):]
                if rank is None or not channel.isdigit():
                    continue
                milliwatt = self.number(label.parent / f'in_power{channel}_input')
                if milliwatt is not None:
                    candidates.append((rank, milliwatt / 1000))
        return min(candidates)[1] if candidates else None


def status_lines(values=None, age=None):
    values = values or {}
    fresh = age is not None and 0 <= age <= 3
    data = values if fresh else {}
    def number(key, suffix='', decimals=1):
        v = data.get(key)
        return '--' if v is None else f'{v:.{decimals}f}' + suffix
    def memory(prefix):
        if prefix + '_used_gib' not in data:
            return '--'
        return f"{data[prefix + '_used_gib']:.2f}/{data[prefix + '_total_gib']:.2f} GiB"
    load = data.get('load')
    load_text = '/'.join(f'{v:.2f}' for v in load) if load else '--'
    tag = 'WAITING' if age is None else 'STALE' if not fresh else f'LIVE ({age:.1f}s ago)'
    return (
        'CPU ' + number('cpu_pct', '%') + ' | GPU ' + number('gpu_pct', '%') +
        ' | CPU cores ' + number('cpu_cores', decimals=0) + ' | ' + tag,
        'RAM ' + memory('ram') + ' | Swap ' + memory('swap') + ' | Disk / free ' + number('disk_free_gib', ' GiB'),
        'Temp CPU ' + number('cpu_temp_c', ' C') + ' | GPU ' + number('gpu_temp_c', ' C') +
        ' | SoC ' + number('soc_temp_c', ' C') + ' | Clock CPU(max policy) ' + number('cpu_mhz', ' MHz', 0) +
        ' | GPU ' + number('gpu_mhz', ' MHz', 0),
        'System load 1/5/15 min ' + load_text + ' | Read-only device totals; -- = unavailable. No START gating.',
    )


class ComputeMonitor:
    def __init__(self):
        self.probe = SystemProbe()
        self.lock = threading.Lock()
        self.values, self.sampled_at = {}, None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name='inspection-compute', daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop.is_set():
            try:
                values = self.probe.sample()
            except Exception as error:
                # Keep CPU/RAM already read if an optional device sensor fails.
                values = dict(self.probe.partial)
                values['error'] = f'{type(error).__name__}: {error}' 
            with self.lock:
                self.values, self.sampled_at = values, time.monotonic()
            self.stop.wait(1.)

    def lines(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            return status_lines(self.values, None if self.sampled_at is None else now - self.sampled_at)

    def gauges(self, now=None):
        """(label, value text, 0..1 or None) for the inspection bars; None hides them."""
        now = time.monotonic() if now is None else now
        with self.lock:
            values, sampled_at = self.values, self.sampled_at
        if sampled_at is None or not 0 <= now - sampled_at <= 3:
            return None
        def percent(key):
            v = values.get(key)
            return ('--', None) if v is None else (f'{v:.0f}%', v / 100)
        used, total = values.get('ram_used_gib'), values.get('ram_total_gib')
        ram = ('--', None) if used is None or not total else (f'{used:.2f}/{total:.2f} GiB', used / total)
        temp = values.get('soc_temp_c')
        soc = ('--', None) if temp is None else (f'{temp:.0f} C', temp / 100)
        return [('CPU', *percent('cpu_pct')), ('GPU', *percent('gpu_pct')), ('RAM', *ram), ('SoC temp', *soc)]

    def close(self):
        self.stop.set()
        self.thread.join(timeout=.2)
