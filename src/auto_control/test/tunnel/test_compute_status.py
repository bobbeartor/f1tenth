from pathlib import Path
import tempfile
import unittest
from auto_control_tunnel.compute_status import SystemProbe, status_lines, ComputeMonitor


class ComputeTests(unittest.TestCase):
    def test_linux_jetson_metrics_and_missing_sensor_clear_previous_value(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            def put(name,value):
                p=root/name; p.parent.mkdir(parents=True,exist_ok=True); p.write_text(value)
            put('proc/stat','cpu 100 0 50 800 50 0 0 0 20 0\ncpu0 1 2 3 4\ncpu1 1 2 3 4\n')
            put('proc/meminfo','MemTotal: 8388608 kB\nMemAvailable: 3145728 kB\nSwapTotal: 2097152 kB\nSwapFree: 1048576 kB\n')
            put('proc/loadavg','1.0 0.5 0.25 1/100 42')
            put('sys/devices/gpu.0/load','475')
            put('sys/class/devfreq/17000000.gpu/cur_freq','612000000')
            put('sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq','1500000')
            put('sys/class/thermal/thermal_zone0/type','cpu-thermal')
            put('sys/class/thermal/thermal_zone0/temp','52000')
            put('sys/class/thermal/thermal_zone1/type','gpu-thermal')
            put('sys/class/thermal/thermal_zone1/temp','49000')
            put('sys/class/thermal/thermal_zone2/type','tj-max-tmargin')
            put('sys/class/thermal/thermal_zone2/temp','80000')
            p=SystemProbe(root/'proc',root/'sys',d)
            a=p.sample()
            self.assertNotIn('cpu_pct',a)
            self.assertEqual(a['gpu_pct'],47.5)
            self.assertEqual(a['gpu_mhz'],612)
            self.assertEqual(a['cpu_mhz'],1500)
            self.assertEqual((a['ram_used_gib'],a['ram_total_gib']),(5,8))
            self.assertEqual((a['swap_used_gib'],a['swap_total_gib']),(1,2))
            self.assertEqual(a['cpu_temp_c'],52)
            self.assertEqual(a['gpu_temp_c'],49)
            self.assertEqual(a['cpu_cores'],2)
            # Guest time changes must not be double-counted as CPU work.
            put('proc/stat','cpu 120 0 60 860 60 0 0 0 99 0\ncpu0 11 2 3 4\ncpu1 1 2 3 14\n')
            b=p.sample()
            self.assertAlmostEqual(b['cpu_pct'],30)
            self.assertEqual(b['cpu_core_pct'], {'cpu0': 100.0, 'cpu1': 0.0})
            (root/'sys/devices/gpu.0/load').unlink()
            self.assertNotIn('gpu_pct',p.sample())
            put('proc/stat','cpu 1 0 0 0 0 0 0 0')
            self.assertNotIn('cpu_pct',p.sample()) # counters reset
            put('sys/devices/gpu.0/load','not a number')
            self.assertNotIn('gpu_pct',p.sample())

    def test_unavailable_and_stale_never_look_like_idle(self):
        with tempfile.TemporaryDirectory() as d:
            p=SystemProbe(Path(d)/'missingproc',Path(d)/'missingsys',d)
            values=p.sample()
            self.assertNotIn('cpu_pct',values)
            self.assertNotIn('gpu_pct',values)
        self.assertIn('CPU -- | GPU --',status_lines()[0])
        self.assertIn('WAITING',status_lines()[0])
        self.assertIn('STALE',status_lines({'gpu_pct':95},age=4)[0])
        self.assertNotIn('95',status_lines({'gpu_pct':95},age=4)[0])
        self.assertIn('GPU 0.0%',status_lines({'gpu_pct':0},age=.1)[0])

    def test_monitor_starts_and_closes_without_changing_controls(self):
        monitor=ComputeMonitor()
        try:
            self.assertEqual(len(monitor.lines()),4)
        finally:
            monitor.close()
        self.assertFalse(monitor.thread.is_alive())


if __name__ == '__main__': unittest.main()


class PowerTests(unittest.TestCase):
    def tree(self, d, files):
        root = Path(d)
        for name, value in files.items():
            p = root / name; p.parent.mkdir(parents=True, exist_ok=True); p.write_text(value)
        return SystemProbe(proc=root / 'proc', sys=root / 'sys', disk=d)

    def test_ina3221_board_input_rail_power(self):
        hw = 'sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon3/'
        with tempfile.TemporaryDirectory() as d:
            probe = self.tree(d, {hw + 'in1_label': 'VDD_IN\n', hw + 'in1_input': '5000',
                                  hw + 'curr1_input': '1400', hw + 'in2_label': 'VDD_CPU_GPU_CV',
                                  hw + 'in2_input': '5000', hw + 'curr2_input': '400'})
            self.assertAlmostEqual(probe.sample()['power_w'], 7.0)

    def test_iio_power_rail_in_milliwatts(self):
        io = 'sys/bus/i2c/drivers/ina3221x/1-0040/iio:device0/'
        with tempfile.TemporaryDirectory() as d:
            probe = self.tree(d, {io + 'rail_name_0': 'POM_5V_IN', io + 'in_power0_input': '6500'})
            self.assertAlmostEqual(probe.sample()['power_w'], 6.5)

    def test_absent_sensor_omits_power(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertNotIn('power_w', self.tree(d, {'proc/stat': ''}).sample())
