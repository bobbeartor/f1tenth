"""remote_preview: numeric telemetry over HTTP/SSE for an iPad/MacBook browser preview.

Read-only toward driving: it only subscribes, and forwards START/STOP to the existing
/auto_control/start and /auto_control/stop Trigger services. It never publishes duty/enable.
"""
import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .compute_status import ComputeMonitor
from .inspection_state import InspectionState
from .remote_frame import RateWindow, build_frame, tunnel_state
from .remote_http import RemoteServer
from .remote_link import LinkWatchdog

# /auto/preview_snapshot key -> frame vehicle key (built by the C++ controller at 30 Hz).
SNAPSHOT_VEHICLE = {
    'speed': 'speed_mps', 'target': 'target_speed_mps', 'duty': 'duty', 'brake': 'brake_current_a',
    'servo': 'servo', 'steer': 'steering_rad', 'raw_steer': 'raw_steering_rad', 'cte': 'cte_m',
    'heading': 'heading_rad', 'curv': 'curvature', 'corner': 'corner_state', 'vesc': 'vesc_connected',
}
TRAFFIC_NAMES = {0: 'UNKNOWN', 1: 'RED', 2: 'GREEN'}
PERF_STALE_SEC = 2.5  # /perf/* stage reports arrive every 1 s


def html_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('auto_control'), 'web', 'remote_preview.html')
        if os.path.exists(path):
            return path
    except Exception:
        pass
    return os.path.join(os.path.dirname(__file__), 'web', 'remote_preview.html')


class RemotePreview(Node):
    def __init__(self):
        super().__init__('remote_preview')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.port = int(p('port', 8765))
        self.bind = str(p('bind', '0.0.0.0'))
        self.lane_width_m = float(p('lane_width_m', 0.65))
        self.frame_rate_hz = float(p('frame_rate_hz', 30.0))
        loss_stop_sec = float(p('link_loss_stop_sec', 1.0))
        expected = list(p('expected_nodes', ['line_detactor']))
        diagnostics_topic = str(p('connection_diagnostics_topic', '/vesc/connected/diagnostics'))
        traffic_topic = str(p('traffic_state_topic', '/traffic_light/state'))
        if not 1.0 <= self.frame_rate_hz <= 60.0 or self.lane_width_m <= 0:
            raise ValueError('frame_rate_hz must be 1..60 and lane_width_m positive')

        self.lock = threading.Lock()
        self.vehicle, self.lane = {}, {}
        self.traffic = self.tunnel_state = self.tunnel_lines = None
        self.perf = {}  # 'camera'/'bev' -> (parsed JSON, received_at)
        self.rates = {key: RateWindow() for key in ('tcn', 'traffic')}
        self.snapshot_stages = ({}, 0.0)
        self.last_traffic_generation = None
        self.message = ''
        self.state = InspectionState(expected)
        self.watchdog = LinkWatchdog(loss_stop_sec)
        self.compute = ComputeMonitor()
        self.last_compute_error = None
        self.seq = 0

        sensor = qos_profile_sensor_data
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Vehicle, lane path and stages 2-4 arrive as one controller summary (no per-topic Python load).
        subs = [self.create_subscription(String, '/auto/preview_snapshot', self.on_snapshot, sensor)]
        perf = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        subs += [
            self.create_subscription(String, '/perf/camera_stage', lambda m: self.on_perf('camera', m), perf),
            self.create_subscription(String, '/perf/bev_stage', lambda m: self.on_perf('bev', m), perf),
            self.create_subscription(String, '/tunnel_tcn/result', self.on_tcn, sensor),
            self.create_subscription(String, '/tunnel/preview_status', self.on_tunnel, latched),
        ]
        try:
            from traffic_detection_test.msg import TrafficLightState
            subs.append(self.create_subscription(TrafficLightState, traffic_topic, self.on_traffic, sensor))
        except ImportError:
            self.get_logger().warn('traffic_detection_test messages unavailable; traffic shown as --')
        subs += [
            self.create_subscription(String, '/bev_processor/startup_status',
                                     lambda m: setattr(self.state, 'camera', m.data), latched),
            self.create_subscription(String, '/auto_control/startup_state',
                                     lambda m: self.state.update_controller(m.data), latched),
            self.create_subscription(String, diagnostics_topic, lambda m: self.state.update_link(m.data), latched),
        ]
        subs += [self.create_subscription(String, f'/{name}/model_status',
                                          lambda m, n=name: self.state.update_model(n, m.data), latched)
                 for name in expected]
        self.subscriptions_owned = subs
        self.start_client = self.create_client(Trigger, '/auto_control/start')
        self.stop_client = self.create_client(Trigger, '/auto_control/stop')

        self.server = RemoteServer(self.bind, self.port, html_path(), self)
        self.server.start()
        self.timer = self.create_timer(1.0 / self.frame_rate_hz, self.tick)
        self.get_logger().info(
            f'Remote preview on http://<jetson-ip>:{self.server.port} ({self.frame_rate_hz:.0f} Hz, '
            f'lane width {self.lane_width_m:.2f} m, link-loss STOP '
            f'{"off" if loss_stop_sec <= 0 else f"{loss_stop_sec:.1f}s"})')

    # ---- ROS callbacks: store numbers only ----
    def on_snapshot(self, message):
        now = time.monotonic()
        try:
            data = json.loads(message.data)
        except ValueError:
            return
        lane = data.get('lane') or {}
        age_ms = lane.get('age_ms')
        lane_at = now - age_ms / 1000.0 if isinstance(age_ms, (int, float)) else now - 10.0
        points = lane.get('points') or []
        stop = lane.get('stop_m')
        with self.lock:
            self.vehicle = {key: (data.get(name), now) for name, key in SNAPSHOT_VEHICLE.items()}
            self.lane = {'points': (points, lane_at), 'valid': (len(points) >= 2, lane_at),
                         'state': (lane.get('state'), lane_at), 'source': (lane.get('source') or 'RGB', lane_at),
                         'stop_line_m': (stop if isinstance(stop, (int, float)) and stop >= 0 else None, lane_at)}
            self.snapshot_stages = (data.get('stages') or {}, now)

    def on_perf(self, key, message):
        try:
            values = json.loads(message.data)
        except ValueError:
            return
        with self.lock:
            self.perf[key] = (values, time.monotonic())

    def on_tcn(self, message):
        try:
            report = json.loads(message.data)
        except ValueError:
            return
        if not str(report.get('status', '')).startswith(('LIVE', 'WARMUP')):
            return
        value = report.get('inference_wall_ms')
        with self.lock:
            self.rates['tcn'].add(time.monotonic(), float(value) if isinstance(value, (int, float)) else None)

    def on_tunnel(self, message):
        now = time.monotonic()
        with self.lock:
            self.tunnel_state = (tunnel_state(message.data), now)
            self.tunnel_lines = (message.data.split('\n')[:3], now)

    def on_traffic(self, message):
        now = time.monotonic()
        with self.lock:
            self.traffic = (TRAFFIC_NAMES.get(int(message.state), 'UNKNOWN'), now)
            if message.source_generation != self.last_traffic_generation:
                self.last_traffic_generation = message.source_generation
                self.rates['traffic'].add(now, float(message.inference_ms))

    # ---- RemoteBackend (called from HTTP threads) ----
    def heartbeat(self):
        self.watchdog.heartbeat(time.monotonic())

    def _call(self, client, name):
        if not client.service_is_ready():
            return False, f'{name} service unavailable'
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(2.0):
            return False, f'{name} timed out (no controller response in 2s)'
        response = future.result()
        return bool(response.success), response.message

    def request_start(self):
        if not self.state.can_start():
            return False, 'Not ready: ' + self.state.controller
        return self._call(self.start_client, 'START')

    def request_stop(self):
        return self._call(self.stop_client, 'STOP')

    # ---- 30 Hz frame ----
    def tick(self):
        now = time.monotonic()
        controller_fresh = self.state.controller_fresh(now)
        started = controller_fresh and self.state.controller.startswith('STARTED')
        if self.watchdog.should_stop(now, started):
            if self.stop_client.service_is_ready():
                self.stop_client.call_async(Trigger.Request())
            self.message = 'LINK LOST: STOP sent — START를 다시 눌러야 출발'
            self.get_logger().warn('Remote preview heartbeat lost while STARTED; STOP requested.')
        elif started:
            self.message = ''
        with self.compute.lock:
            system = dict(self.compute.values) if self.compute.sampled_at and now - self.compute.sampled_at < 3 else {}
        error = system.get('error')
        if error and error != self.last_compute_error:
            self.get_logger().warning(f'System metrics: {error}')
        self.last_compute_error = error
        with self.lock:
            perf = {key: value for key, (value, at) in self.perf.items() if now - at < PERF_STALE_SEC}
            camera, bev = perf.get('camera', {}), perf.get('bev', {})
            stages = {key: rate.stats(now) for key, rate in self.rates.items()}
            controller, at = self.snapshot_stages
            if now - at < PERF_STALE_SEC:
                stages.update({key: self._perf_stage(controller.get(key)) for key in ('infer', 'post', 'control', 'valid_lane')})
            stages.update({
                'capture': self._perf_stage(camera.get('capture')),
                'stabilize': self._perf_stage(camera.get('stabilize')),
                'bev': self._perf_stage(bev.get('bev')),
            })
            snap = {
                'vehicle': dict(self.vehicle), 'lane': dict(self.lane), 'lane_width_m': self.lane_width_m,
                'stages': stages,
                'traffic': self.traffic,
                'tunnel': {'state': self.tunnel_state, 'lines': self.tunnel_lines},
            }
        snap['system'] = system
        snap['startup'] = {'camera': self.state.camera, 'models': dict(self.state.models),
                           'controller': self.state.controller if controller_fresh else 'WAITING | controller silent',
                           'can_start': self.state.can_start(now), 'started': started, 'message': self.message}
        snap['link'] = {'clients': self.server.client_count(),
                        'heartbeat_age_ms': self.watchdog.heartbeat_age_ms(now),
                        'loss_stop_sec': self.watchdog.timeout_sec}
        self.seq += 1
        self.server.broadcast(build_frame(self.seq, now, snap))

    @staticmethod
    def _perf_stage(value):
        if not isinstance(value, dict):
            return {}
        return {'hz': value.get('hz'), 'ms': value.get('avg_ms'), 'max_ms': value.get('max_ms')}

    def destroy_node(self):
        self.server.close()
        self.compute.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RemotePreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
