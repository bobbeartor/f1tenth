"""Main-thread OpenCV startup inspection; never publishes actuator commands."""
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .inspection_state import InspectionState
from .inspection_speed import InspectionSpeed
from .compute_status import ComputeMonitor, status_lines


DISPLAY_SCALE = .85
START = (25, 655, 740, 720)
STOP = (760, 655, 975, 720)
TIMINGS = (25, 740, 200, 790)
PARAMS = (215, 740, 390, 790)
SPEED = (415, 735, 1175, 790)
SPEED_LEFT, SPEED_RIGHT, SPEED_Y = 440, 1145, 775


def contains(rect, x, y):
    return rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]


class StartupInspection(Node):
    def __init__(self):
        super().__init__('startup_inspection')
        expected = self.declare_parameter('expected_nodes', ['line_detactor']).value
        self.state = InspectionState(expected)
        self.image = None
        self.image_at = 0.
        self.pending = None
        self.speed = InspectionSpeed()
        from std_msgs.msg import Float64
        from line_detactor.msg import SpeedLimitState
        self.tool_clients = {
            'metrics': self.create_client(Trigger, '/auto/runtime_tools/open_metrics'),
            'parameters': self.create_client(Trigger, '/auto/runtime_tools/open_parameters'),
        }
        self.pending_tool = None
        self.speed_pub = self.create_publisher(Float64, '/auto/maximum_speed_request', 1)
        self.speed_sub = self.create_subscription(
            SpeedLimitState, '/auto/maximum_speed_state', self.speed.observe,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        diagnostics_topic = self.declare_parameter(
            'connection_diagnostics_topic', '/vesc/connected/diagnostics').value
        self.link_diagnostics_sub = self.create_subscription(
            String, diagnostics_topic, lambda message: self.state.update_link(message.data), qos)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_owned = [
            self.create_subscription(String, '/bev_processor/startup_status', self.on_camera, qos),
            self.create_subscription(String, '/auto_control/startup_state', self.on_controller, qos),
            self.create_subscription(Image, '/bev_processor/startup_preview', self.on_image, image_qos),
        ]
        self.subscriptions_owned += [self.create_subscription(
            String, f'/{name}/model_status', lambda msg, name=name: self.on_model(name, msg), qos)
            for name in expected]
        self.client = self.create_client(Trigger, '/auto_control/start')
        self.stop_client = self.create_client(Trigger, '/auto_control/stop')
        self.compute = ComputeMonitor()

    def destroy_node(self):
        self.compute.close()
        return super().destroy_node()

    def on_camera(self, message):
        self.state.camera = message.data

    def on_model(self, name, message):
        self.state.update_model(name, message.data)

    def on_controller(self, message):
        self.state.update_controller(message.data)

    def on_image(self, message):
        import numpy as np
        if (message.encoding != 'bgr8' or message.height <= 0 or message.width <= 0
                or message.step < message.width * 3 or len(message.data) < message.step * message.height):
            return
        rows = np.frombuffer(message.data, dtype=np.uint8,
            count=message.height*message.step).reshape(message.height, message.step)
        self.image = rows[:, :message.width * 3].reshape(message.height, message.width, 3).copy()
        self.image_at = time.monotonic()

    def on_mouse(self, event, x, y, flags, userdata):
        import cv2
        # HighGUI receives coordinates in the displayed (85%) image, not the
        # logical 1200x1020 drawing. Keep drawing and hit targets in one space.
        x, y = x / DISPLAY_SCALE, y / DISPLAY_SCALE
        if event == cv2.EVENT_LBUTTONDOWN and contains(SPEED, x, y):
            if self.speed.fresh(time.monotonic()):
                self.speed.dragging = True
                self.speed.select(x, SPEED_LEFT, SPEED_RIGHT)
        elif event == cv2.EVENT_MOUSEMOVE and self.speed.dragging:
            if flags & cv2.EVENT_FLAG_LBUTTON:
                self.speed.select(x, SPEED_LEFT, SPEED_RIGHT)
            else:
                self.speed.dragging = False
                self.speed.released = self.speed.dirty
        elif event == cv2.EVENT_LBUTTONUP:
            if self.speed.dragging:
                self.speed.select(x, SPEED_LEFT, SPEED_RIGHT)
                self.speed.dragging = False
                self.speed.released = self.speed.dirty
                return
            if contains(START, x, y):
                self.request_start()
            elif contains(STOP, x, y):
                self.request_stop()
            elif contains(TIMINGS, x, y):
                self.request_tool('metrics')
            elif contains(PARAMS, x, y):
                self.request_tool('parameters')

    def request_tool(self, key):
        if self.pending_tool is not None:
            self.state.error = 'Waiting for tools window response...'
            return
        client = self.tool_clients[key]
        if not client.service_is_ready():
            self.state.error = 'Tools unavailable: check runtime_tools startup and python3-tk installation.'
            return
        try:
            future = client.call_async(Trigger.Request())
        except Exception as error:
            self.state.error = f'Tools request failed: {error}'
            return
        self.pending_tool = (future, time.monotonic())
        self.state.error = f'Opening {"Timings" if key == "metrics" else "Parameters"}...'
        future.add_done_callback(self.tool_response)

    def tool_response(self, future):
        if self.pending_tool is None or self.pending_tool[0] is not future:
            return
        self.pending_tool = None
        try:
            response = future.result()
            self.state.error = response.message if response.success else 'Tools error: ' + response.message
        except Exception as error:
            self.state.error = f'Tools error: {error}'

    def check_tool_timeout(self):
        if self.pending_tool is not None and time.monotonic()-self.pending_tool[1] > 10.:
            future = self.pending_tool[0]
            self.pending_tool = None
            # Do not replay this click if the tools process becomes available later.
            future.cancel()
            self.state.error = 'Tools did not respond within 10s; check runtime_tools process.'

    def flush_speed_request(self):
        from std_msgs.msg import Float64
        value = self.speed.request(time.monotonic())
        if value is not None:
            self.speed_pub.publish(Float64(data=value))

    def request_stop(self):
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request()).add_done_callback(self.on_stop_response)
        else:
            self.state.error = 'STOP service unavailable'

    def on_stop_response(self, future):
        try:
            self.state.error = future.result().message
        except Exception as error:
            self.state.error = f'STOP failed: {error}'

    def request_start(self):
        if not self.state.can_start() or self.pending is not None:
            return
        if not self.client.service_is_ready():
            self.state.error = 'Controller START service unavailable; duty stays blocked.'
            return
        self.pending = self.client.call_async(Trigger.Request())
        self.pending.add_done_callback(self.on_response)

    def on_response(self, future):
        self.pending = None
        try:
            response = future.result()
            self.state.error = response.message
        except Exception as error:
            self.state.error = f'START failed: {error}'


# Daylight theme, BGR. Light ground with dark text stays readable outdoors.
THEME = dict(
    bg=(236,240,238), panel=(255,255,255), border=(193,201,196), text=(26,24,20), dim=(100,96,86),
    accent=(184,79,28), ok=(60,122,15), warn=(0,94,168), bad=(29,38,194), start=(63,129,18),
    idle=(168,176,170), stop=(29,38,194), tool=(221,228,224), track=(218,225,221), error=(26,67,168),
    on_fill=(255,255,255))


def render(state, image=None, image_fresh=False, pending=False, now=None, speed=None, compute=None, gauges=None):
    import cv2
    import numpy as np
    import textwrap
    now = time.monotonic() if now is None else now
    t = THEME
    font = cv2.FONT_HERSHEY_SIMPLEX
    canvas = np.full((1020,1200,3), t['bg'], dtype=np.uint8)
    def text(line, x, y, color=t['text'], scale=.55, thickness=1):
        cv2.putText(canvas, line[:112], (x,y), font, scale, color, thickness, cv2.LINE_AA)
    def centered(line, rect, color, scale, thickness=1):
        (width, height), _ = cv2.getTextSize(line, font, scale, thickness)
        text(line, (rect[0]+rect[2]-width)//2, (rect[1]+rect[3]+height)//2, color, scale, thickness)
    def pill(line, right, top, color):
        (width, height), _ = cv2.getTextSize(line, font, .45, 1)
        cv2.rectangle(canvas, (right-width-16,top), (right,top+22), color, -1)
        text(line, right-width-8, top+11+height//2, t['on_fill'], .45)
    def card(rect, number, title):
        cv2.rectangle(canvas, rect[:2], rect[2:], t['panel'], -1)
        cv2.rectangle(canvas, rect[:2], rect[2:], t['border'], 1)
        cv2.line(canvas, (rect[0],rect[1]+38), (rect[2],rect[1]+38), t['border'], 1)
        text(number, rect[0]+14, rect[1]+26, t['accent'], .6, 2)
        text(title, rect[0]+36, rect[1]+26, scale=.6, thickness=2)
    def status_color(good, failed):
        return t['ok'] if good else t['bad'] if failed else t['warn']

    text('STARTUP INSPECTION', 22,38, scale=.9, thickness=2)

    card((20,56,510,514), '1', 'CAMERA ATTITUDE')
    passed = state.camera.startswith('READY | ')
    camera_failed = state.camera.startswith('FAILED')
    color = status_color(passed, camera_failed)
    pill('PASS' if passed else 'FAILED' if camera_failed else 'MEASURING', 498, 64, color)
    if image is not None:
        canvas[100:400,25:505] = cv2.resize(image, (480,300))
        text('Live measurement view' if image_fresh else 'Last measurement view (frozen)', 30,422, t['dim'], .45)
    else:
        cv2.rectangle(canvas, (25,100), (505,400), t['track'], -1)
        text('Waiting for camera measurement view...', 95,255, t['dim'])
    detail = state.camera.split(' | ',1)[-1]
    for row, line in enumerate(textwrap.wrap(detail, width=53)[:4]):
        text(line,30,446+row*19,color if not passed else t['text'],.41)

    card((525,56,1180,514), '2', 'INFERENCE MODELS')
    ready_count = sum(value.startswith('READY | ') for value in state.models.values())
    pill(f'{ready_count}/{len(state.models)} READY', 1168, 64,
         t['ok'] if state.models and ready_count == len(state.models) else t['warn'])
    for index, name in enumerate(state.models):
        info = state.model_info(name, now)
        tag = info['tag']
        y = 118 + index * 98
        color = status_color(tag=='READY', tag=='FAILED' or 'STALE' in info['health'])
        text(name,540,y,scale=.54,thickness=2)
        pill(tag, 1166, y-17, color)
        phase = info['phase']
        if tag == 'READY':
            phase = 'Engine loaded; trial inference PASS'
        elif phase.startswith('model='):
            phase = 'Opening model: ' + os.path.basename(phase[6:])
        for row, line in enumerate(textwrap.wrap(phase, width=77)[:2]):
            text(line,540,y+19+row*16,scale=.40)
        text(f"Total {info['elapsed']:.0f}s | Stage {info['stage_age']:.0f}s | Last report {info['age']:.1f}s ago",
             540,y+52,t['dim'],.40)
        text(info['health'],540,y+68,color,.40)
        cv2.rectangle(canvas,(540,y+76),(1166,y+79),color if tag=='READY' else t['track'],-1)
    text('Stage changes reflect loader activity; a heartbeat alone cannot prove GPU progress.',540,507,t['dim'],.37)

    card((20,526,1180,844), '3', 'DEPARTURE')
    controller = state.controller if state.controller_fresh(now) else 'WAITING | controller heartbeat unavailable'
    if 'last stop: VESC disconnected' in controller and state.link_fault:
        detail = state.link_fault
        if state.link_recovered:
            detail += '; UART recovered; START reapproval required'
        controller = controller.replace('last stop: VESC disconnected', 'last stop: ' + detail)
    ready = state.can_start(now) and not pending
    active = state.completed(now)
    pill(controller.split(' | ',1)[0][:24], 1168, 534, t['ok'] if ready or active else t['warn'])
    for row, line in enumerate(textwrap.wrap(controller, width=112)[:2]):
        text(line,34,586+row*18,scale=.48)
    text('The controller checks models, measured camera pose, VESC feedback and a fresh valid lane.',34,622,t['dim'],.43)
    text('Automatic duty is blocked until an accepted START request. Manual mode skips this button.',34,640,t['dim'],.43)
    cv2.rectangle(canvas,START[:2],START[2:],t['start'] if ready else t['idle'],-1)
    label = ('MANUAL CONTROL' if active and state.controller.startswith('MANUAL | ') else
             'DRIVING / START ACCEPTED' if active else 'START AUTOMATIC DRIVING' if ready else
             'REQUESTING START...' if pending else 'WAITING FOR CHECKS')
    centered(label,START,t['on_fill'],.78,2)
    cv2.rectangle(canvas,STOP[:2],STOP[2:],t['stop'],-1)
    centered('STOP',STOP,t['on_fill'],.78,2)
    for rect, label in ((TIMINGS, 'Timings'), (PARAMS, 'Params')):
        cv2.rectangle(canvas,rect[:2],rect[2:],t['tool'],-1)
        cv2.rectangle(canvas,rect[:2],rect[2:],t['border'],1)
        centered(label,rect,t['text'],.62)
    speed = speed or dict(bounds=None, selected=None, applied=None, status='CONTROLLER OFFLINE')
    if speed['bounds'] is not None:
        minimum, ceiling = speed['bounds']
        selected, applied = speed['selected'], speed['applied']
        text(f'MAX SPEED {selected:.2f} m/s | Applied {applied:.2f}',SPEED_LEFT,756,scale=.45)
        pill(speed['status'][:24], SPEED_RIGHT, 739, t['ok'] if speed['status']=='LIVE' else t['warn'])
        ratio = (selected-minimum)/(ceiling-minimum) if ceiling > minimum else 0.
        thumb = round(SPEED_LEFT+ratio*(SPEED_RIGHT-SPEED_LEFT))
        cv2.line(canvas,(SPEED_LEFT,SPEED_Y),(SPEED_RIGHT,SPEED_Y),t['track'],7)
        cv2.line(canvas,(SPEED_LEFT,SPEED_Y),(thumb,SPEED_Y),t['accent'],7)
        cv2.circle(canvas,(thumb,SPEED_Y),9,t['accent'],-1)
        text(f'{minimum:.2f}',SPEED_LEFT,802,t['dim'],.36)
        text(f'{ceiling:.2f} m/s',1085,802,t['dim'],.36)
    else:
        text('MAX SPEED: waiting for controller',SPEED_LEFT,770,t['dim'],.48)
    text(state.error,34,820,t['error'],.43)
    text('START requires checks. STOP, Timings, Params and speed controls remain available here after START.',34,838,t['dim'],.40)

    card((20,856,1180,1010), '4', 'COMPUTING STATUS (1 Hz)')
    lines = tuple(compute or status_lines())
    tag = lines[0].rsplit(' | ',1)[-1]
    pill(tag[:24], 1168, 864, t['ok'] if tag.startswith('LIVE') else t['warn'])
    if gauges:
        for index, (label, value, ratio) in enumerate(gauges[:4]):
            x = 34 + index * 288
            (width, _), _ = cv2.getTextSize(value, font, .45, 1)
            text(label,x,916,t['dim'],.42)
            text(value,x+262-width,916,scale=.45)
            cv2.rectangle(canvas,(x,924),(x+262,931),t['track'],-1)
            if ratio is not None:
                filled = round(262*min(max(ratio,0.),1.))
                cv2.rectangle(canvas,(x,924),(x+filled,931),t['warn'] if ratio >= .85 else t['accent'],-1)
        for index, line in enumerate(lines[1:]):
            text(line,34,955+index*20,t['text'] if index < 2 else t['dim'],.40)
    else:
        for index, line in enumerate(lines):
            text(line,34,915+index*24,scale=.43)
    return cv2.resize(canvas, (round(canvas.shape[1]*DISPLAY_SCALE), round(canvas.shape[0]*DISPLAY_SCALE)),
                      interpolation=cv2.INTER_AREA)


def main(args=None):
    rclpy.init(args=args)
    node = StartupInspection()
    shown = False
    dismissed = False
    last_render = 0.
    window = 'Safety inspection and driving controls'
    gui = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY') or os.sys.platform=='darwin')
    cv2 = None
    if gui:
        try:
            import cv2
        except ImportError as error:
            node.get_logger().error(f'Inspection GUI unavailable: {error}; automatic duty stays blocked.')
            gui = False
    else:
        node.get_logger().error('No display for startup inspection. Automatic duty requires an explicit controller START service request.')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.03)
            node.flush_speed_request()
            node.check_tool_timeout()
            if not gui:
                continue
            if dismissed:
                continue
            if not shown:
                cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
                cv2.setMouseCallback(window, node.on_mouse)
                shown = True
            # Redrawing the whole panel dominated this process. 10 Hz is
            # enough for status; follow the speed slider at full rate.
            now = time.monotonic()
            if now - last_render >= (.03 if node.speed.dragging else .1):
                last_render = now
                cv2.imshow(window, render(node.state,node.image,
                    now-node.image_at<.5,node.pending is not None,
                    speed=node.speed.view(now), compute=node.compute.lines(),
                    gauges=node.compute.gauges()))
            key = cv2.waitKey(1)&255
            if key in (10,13):
                node.request_start()
            if key in (ord('q'),27) or cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1:
                cv2.destroyWindow(window)
                shown = False
                dismissed = True
                node.speed.dragging = node.speed.dirty = node.speed.released = False
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().error(f'Inspection GUI stopped: {error}. No automatic START approval was issued by this error.')
    finally:
        if shown and cv2 is not None:
            cv2.destroyWindow(window)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
