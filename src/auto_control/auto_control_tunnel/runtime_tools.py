"""On-demand Tk windows. ROS work is asynchronous on the GUI event loop."""
import json
import math
import os
import time
from pathlib import Path

from .parameter_preset import parameter_values, write_preset

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from .remote_parameters import RemoteParameters
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def category(name):
    if 'fps' in name or name.endswith('rate_hz'):
        return 'FPS·주기'
    if name.startswith('obstacles.'):
        return '장애물·회피'
    if name.startswith('traffic_'):
        return '신호등·정지선'
    if name.startswith(('stanley_', 'steering_', 'maximum_steering', 'stop_steering')):
        return '차선·조향'
    if name.startswith(('corner_', 'curvature_', 'maximum_lateral')):
        return '코너 속도'
    if name.startswith(('path_', 'erpm_timeout')):
        return '경로·입력 유효성'
    if name.startswith(('brake_', 'electrical_')):
        return '브레이크'
    if name.startswith(('speed_pid_', 'longitudinal_pid_')):
        return '속도 PID'
    if name.startswith(('speed_', 'maximum_speed', 'minimum_speed', 'duty_', 'maximum_duty',
                        'minimum_duty', 'longitudinal_', 'obstacle_')):
        return '가감속·속도'
    return '기타·진단'


def numeric(value, kind):
    v = float(value)
    if not math.isfinite(v):
        raise ValueError('유한한 숫자를 입력하세요.')
    if kind == Parameter.Type.BOOL:
        if v not in (0., 1.):
            raise ValueError('ON/OFF는 0 또는 1입니다.')
        return bool(v)
    if kind == Parameter.Type.INTEGER:
        if not v.is_integer():
            raise ValueError('정수로 입력하세요.')
        return int(v)
    return v


class RuntimeTools(Node):
    def __init__(self, root):
        super().__init__('auto_runtime_tools')
        from tkinter import ttk
        self.root = root
        self.ttk = ttk
        self.windows = {}
        # Node.clients is a read-only ROS entity iterator; keep our lookup separate.
        self.parameter_clients = {}
        self.latest = {}
        self.caps = {}
        self.fields = {}
        self.pending_nodes = set()
        self.preset_request = None
        self.metrics_pub = self.create_publisher(
            Bool, '/auto/runtime_metrics/enabled',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/auto/ui/action', self.action, 10)
        self.open_metrics_service = self.create_service(
            Trigger, '/auto/runtime_tools/open_metrics',
            lambda request, response: self.open_tool('metrics', response))
        self.open_parameters_service = self.create_service(
            Trigger, '/auto/runtime_tools/open_parameters',
            lambda request, response: self.open_tool('parameters', response))
        self.create_subscription(String, '/auto/runtime_metrics', self.metric, 30)
        self.stop_client = self.create_client(Trigger, '/auto_control/stop')
        self.start_client = self.create_client(Trigger, '/auto_control/start')
        self.set_metrics(False)
        self.last_heartbeat = 0.

    def set_metrics(self, enabled):
        self.metrics_pub.publish(Bool(data=enabled))

    def open_tool(self, key, response):
        try:
            if key == 'metrics':
                self.show_metrics()
            else:
                self.show_parameters()
            window = self.windows[key]
            window.deiconify()
            window.update_idletasks()
            window.lift()
            window.focus_force()
            # A click explicitly asks for this window; bring it above the inspection.
            window.attributes('-topmost', True)
            window.after(250, lambda: self.release_topmost(window))
            response.success = True
            response.message = f'{"Timings" if key == "metrics" else "Parameters"} window opened.'
        except Exception as error:
            response.success = False
            response.message = f'Cannot open {key}: {error}'
            self.get_logger().error(response.message)
            if key in self.windows:
                try:
                    self.close(key)
                except Exception:
                    pass
        return response

    @staticmethod
    def release_topmost(window):
        try:
            if window.winfo_exists():
                window.attributes('-topmost', False)
        except Exception:
            pass  # The user may close the window before the deferred callback.

    def action(self, msg):
        if msg.data == 'metrics':
            self.show_metrics()
        elif msg.data == 'parameters':
            self.show_parameters()
        elif msg.data == 'stop':
            self.request(self.stop_client)

    def request(self, client):
        if not client.service_is_ready():
            self.get_logger().error(f'Unavailable service: {client.srv_name}')
            return
        client.call_async(Trigger.Request()).add_done_callback(self.report)

    def report(self, future):
        try:
            result = future.result()
            self.get_logger().info(result.message)
        except Exception as error:
            self.get_logger().error(str(error))

    def window(self, key, title):
        import tkinter as tk
        if key in self.windows:
            self.windows[key].deiconify()
            self.windows[key].lift()
            return None
        window = tk.Toplevel(self.root)
        window.title(title)
        window.geometry('1120x650')
        window.protocol('WM_DELETE_WINDOW', lambda: self.close(key))
        self.windows[key] = window
        return window

    def close(self, key):
        if key == 'parameters':
            self.preset_request = None
        self.windows.pop(key).destroy()
        if key == 'metrics':
            self.set_metrics(False)
            self.latest.clear()

    def show_metrics(self):
        window = self.window('metrics', '단계별 연산량 · 창이 열려 있을 때 측정')
        if window is None:
            return
        self.latest.clear()
        self.read_caps()
        self.set_metrics(True)
        self.ttk.Label(window, text='실제 완료 FPS / 평균·p95 연산 시간 / 연산 시간 기준 추정 처리능력'
                       '    ·    2초 이상 결과가 없으면 STALE').pack(pady=10)
        columns = ('node', 'stage', 'cap', 'fps', 'mean', 'p95', 'capacity', 'state')
        self.table = self.ttk.Treeview(window, columns=columns, show='headings')
        for key, label in zip(columns, ('노드', '처리 단계', '설정 상한', '실제 FPS', '평균 ms',
                                        'p95 ms', '추정 FPS', '상태')):
            self.table.heading(key, text=label)
            self.table.column(key, width=125)
        self.table.pack(fill='both', expand=True)
        self.ttk.Label(window, text='rgb_capture는 촬영 관측 빈도입니다. 촬영 연산 시간은 호스트에서 측정할 수 없습니다.'
                       '\n추정 FPS = 1000 / 평균 ms이며 실제 완료 FPS와 다릅니다. 기존 제어·추론용 타이밍은 계속 사용됩니다.').pack(pady=8)

    def parameter_client(self, name):
        if name not in self.parameter_clients:
            self.parameter_clients[name] = RemoteParameters(self, name)
        return self.parameter_clients[name]

    def read_caps(self):
        requests = {
            '/camera_driver': {'sensor_fps': ('rgb_capture',), 'publish_fps': ('camera_to_bev',)},
            '/traffic_light_detector': {'inference_fps': ('traffic_inference',)},
            '/auto_obstacles': {'obstacles.avoidance.max_fps': ('avoidance_plan',)},
        }
        for node, stages in requests.items():
            client = self.parameter_client(node)
            if not client.services_are_ready():
                continue
            def received(future, stages=stages):
                try:
                    for name, value in zip(stages, future.result().values):
                        cap = value.double_value if value.type == 3 else value.integer_value
                        for stage in stages[name]:
                            self.caps[stage] = f'{cap:g}'
                except Exception as error:
                    self.get_logger().warning(str(error))
            client.get_parameters(list(stages)).add_done_callback(received)

    def metric(self, msg):
        if 'metrics' not in self.windows:
            return
        try:
            row = json.loads(msg.data)
            self.latest[(row['node'], row['stage'])] = (time.monotonic(), row)
        except (ValueError, KeyError, TypeError):
            pass

    def show_parameters(self):
        window = self.window('parameters', 'auto_control 실시간 파라미터 · 숫자 입력')
        if window is None:
            return
        import tkinter as tk
        self.parameter_nodes = set()
        self.parameter_loading = set()
        self.parameter_errors = set()
        self.preset_status = tk.StringVar(value='파라미터를 불러오는 중…')
        toolbar = self.ttk.Frame(window)
        toolbar.pack(fill='x', padx=8, pady=6)
        self.preset_button = self.ttk.Button(
            toolbar, text='프리셋 저장', command=self.save_preset, state='disabled')
        self.preset_button.pack(side='left')
        self.ttk.Label(toolbar, textvariable=self.preset_status, wraplength=900).pack(side='left', padx=10)
        self.ttk.Label(window, text='기능별 탭 · BOOL은 0/1 · Apply는 해당 탭의 변경분을 노드 단위로 일괄 검증'
                       '\n프리셋 저장은 모든 연결 노드의 전체 적용값을 YAML로 저장합니다. 미적용 입력은 제외됩니다.').pack(pady=8)
        self.tabs = self.ttk.Notebook(window)
        self.tabs.pack(fill='both', expand=True)
        self.fields = {}
        self.tab_choices = {}
        self.tab_selector = self.ttk.Combobox(window, state='readonly')
        self.tab_selector.pack(fill='x', before=self.tabs, padx=8, pady=4)
        self.tab_selector.bind('<<ComboboxSelected>>',
            lambda event: self.tabs.select(self.tab_choices[self.tab_selector.get()]))
        for name in ('/auto_control', '/auto_obstacles', '/camera_driver', '/bev_processor',
                     '/line_detactor', '/traffic_light_detector'):
            client = self.parameter_client(name)
            if not client.services_are_ready():
                self.get_logger().warning(f'Parameter service unavailable: {name}')
                continue
            self.parameter_loading.add(name)
            client.list_parameters(depth=0).add_done_callback(
                lambda future, name=name: self.listed(name, future))
        self.update_preset_status()

    def update_preset_status(self):
        if 'parameters' not in self.windows:
            return
        ready = (self.parameter_nodes and not self.parameter_loading
                 and not self.parameter_errors and self.preset_request is None)
        self.preset_button.configure(state='normal' if ready else 'disabled')
        if self.parameter_loading:
            self.preset_status.set('파라미터를 불러오는 중…')
        elif self.parameter_errors:
            self.preset_status.set('읽기 실패 · 창을 다시 여세요: ' + ', '.join(sorted(self.parameter_errors)))
        else:
            self.preset_status.set('저장 대상: ' + ', '.join(sorted(self.parameter_nodes)))

    def save_preset(self):
        from tkinter import filedialog
        if self.preset_request is not None:
            return
        if self.pending_nodes:
            self.preset_status.set('Apply 완료 후 프리셋을 저장하세요.')
            return
        if not self.parameter_nodes or self.parameter_loading or self.parameter_errors:
            return
        try:
            directory = Path.home() / '.ros' / 'auto_control' / 'presets'
            directory.mkdir(parents=True, exist_ok=True)
            path = filedialog.asksaveasfilename(
                parent=self.windows['parameters'], title='현재 적용값 프리셋 저장',
                initialdir=str(directory),
                initialfile=time.strftime('auto_control_%Y%m%d_%H%M%S.yaml'),
                defaultextension='.yaml', filetypes=[('ROS parameter YAML', '*.yaml')])
            if not path:
                return
            request = {'path': path, 'remaining': set(self.parameter_nodes), 'snapshot': {}}
            self.preset_request = request
            self.preset_button.configure(state='disabled')
            self.preset_status.set('전체 적용값을 읽는 중…')
            self.root.after(10000, lambda: self.preset_failed(request, '파라미터 읽기 시간 초과'))
            for node in sorted(self.parameter_nodes):
                client = self.parameter_clients[node]
                if not client.services_are_ready():
                    raise RuntimeError(f'파라미터 서비스 연결 끊김: {node}')
                client.list_parameters(depth=0).add_done_callback(
                    lambda future, n=node: self.preset_listed(request, n, future))
        except Exception as error:
            if self.preset_request is not None:
                self.preset_failed(self.preset_request, str(error))
            else:
                self.preset_status.set('저장 실패 · ' + str(error))

    def preset_failed(self, request, reason):
        if self.preset_request is not request:
            return
        self.preset_request = None
        if 'parameters' in self.windows:
            self.update_preset_status()
            self.preset_status.set('저장 실패 · ' + reason)

    def preset_listed(self, request, node, future):
        if self.preset_request is not request:
            return
        try:
            names = list(future.result().result.names)
            if not names:
                raise ValueError(f'파라미터가 없습니다: {node}')
            self.parameter_clients[node].get_parameters(names).add_done_callback(
                lambda f: self.preset_loaded(request, node, names, f))
        except Exception as error:
            self.preset_failed(request, str(error))

    def preset_loaded(self, request, node, names, future):
        if self.preset_request is not request:
            return
        try:
            request['snapshot'][node] = parameter_values(names, future.result().values)
            request['remaining'].remove(node)
            if not request['remaining']:
                write_preset(request['path'], request['snapshot'])
                self.preset_request = None
                self.update_preset_status()
                self.preset_status.set('저장 완료 · ' + request['path'])
        except Exception as error:
            self.preset_failed(request, str(error))

    def listed(self, node, future):
        if 'parameters' not in self.windows:
            return
        try:
            names = future.result().result.names
            self.parameter_clients[node].get_parameters(names).add_done_callback(
                lambda f: self.loaded(node, names, f))
        except Exception as error:
            self.parameter_loading.discard(node)
            self.parameter_errors.add(node)
            self.update_preset_status()
            self.get_logger().error(str(error))

    def loaded(self, node, names, future):
        import tkinter as tk
        if 'parameters' not in self.windows:
            return
        try:
            values = dict(zip(names, future.result().values))
            editable = set(values['runtime_editable_parameters'].string_array_value) if 'runtime_editable_parameters' in values else set()
            grouped = {}
            for name, value in values.items():
                kind = Parameter.Type(value.type)
                if kind not in (Parameter.Type.BOOL, Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
                    continue
                grouped.setdefault(category(name), []).append((name, value, kind))
            for group, entries in sorted(grouped.items()):
                frame = self.ttk.Frame(self.tabs)
                label = f'{node.lstrip("/")} / {group}'
                self.tabs.add(frame, text=label)
                self.tab_choices[label] = frame
                self.tab_selector['values'] = tuple(self.tab_choices)
                status = tk.StringVar(value='값을 수정하고 Apply를 누르세요.')
                self.ttk.Label(frame, textvariable=status).pack()
                canvas = tk.Canvas(frame, highlightthickness=0)
                scrollbar = self.ttk.Scrollbar(frame, orient='vertical', command=canvas.yview)
                canvas.configure(yscrollcommand=scrollbar.set)
                scrollbar.pack(side='right', fill='y')
                canvas.pack(side='top', fill='both', expand=True)
                body = self.ttk.Frame(canvas)
                canvas.create_window((0, 0), window=body, anchor='nw')
                body.bind('<Configure>', lambda e, c=canvas: c.configure(scrollregion=c.bbox('all')))
                rows = []
                for index, (name, value, kind) in enumerate(sorted(entries)):
                    current = (value.bool_value if kind == Parameter.Type.BOOL else
                               value.integer_value if kind == Parameter.Type.INTEGER else value.double_value)
                    text = str(int(current)) if kind == Parameter.Type.BOOL else str(current)
                    var = tk.StringVar(value=text)
                    self.ttk.Label(body, text=name, width=65).grid(row=index, column=0, padx=8, pady=4, sticky='w')
                    self.ttk.Entry(body, textvariable=var, width=18,
                                   state='normal' if name in editable else 'readonly').grid(row=index, column=1)
                    self.ttk.Label(body, text='LIVE' if name in editable else '재시작 필요').grid(row=index, column=2, padx=10)
                    if name in editable:
                        rows.append([name, var, kind, current])
                button = self.ttk.Button(frame, text='Apply',
                    command=lambda n=node, r=rows, st=status: self.apply(n, r, st))
                button.pack(pady=6)
        except Exception as error:
            self.parameter_errors.add(node)
            self.get_logger().error(str(error))
        else:
            self.parameter_nodes.add(node)
        finally:
            self.parameter_loading.discard(node)
            self.update_preset_status()

    def apply(self, node, rows, status):
        if self.preset_request is not None:
            status.set('프리셋 저장 완료 후 적용하세요.')
            return
        if node in self.pending_nodes:
            status.set('앞선 적용 결과를 기다리는 중입니다.')
            return
        try:
            changed = [(row, numeric(row[1].get(), row[2])) for row in rows]
            changed = [(row, value) for row, value in changed if value != row[3]]
            if not changed:
                status.set('변경된 값이 없습니다.')
                return
            request = [Parameter(row[0], row[2], value) for row, value in changed]
            self.pending_nodes.add(node)
            status.set('검증·적용 중…')
            self.parameter_clients[node].set_parameters_atomically(request).add_done_callback(
                lambda future: self.applied(node, future, changed, status))
        except Exception as error:
            status.set(str(error))

    def applied(self, node, future, changed, status):
        self.pending_nodes.discard(node)
        if 'parameters' not in self.windows:
            return
        try:
            reply = future.result().result
            if reply.successful:
                for row, value in changed:
                    row[3] = value
                status.set('적용 완료')
                if 'metrics' in self.windows:
                    self.read_caps()
            else:
                status.set('거절 · ' + reply.reason)
        except Exception as error:
            status.set(str(error))

    def tick(self):
        try:
            self.poll()
        except Exception as error:
            self.get_logger().error(f'Runtime tools update failed: {error}')
        finally:
            # A Tk/ROS callback error must not silently stop all future requests.
            if rclpy.ok():
                # Fast polling only while a tool window is in use; the idle
                # process otherwise just waits for an open request.
                self.root.after(10 if self.windows else 50, self.tick)

    def poll(self):
        rclpy.spin_once(self, timeout_sec=0)
        now = time.monotonic()
        if 'metrics' in self.windows:
            if now - self.last_heartbeat > .5:
                self.set_metrics(True)
                self.last_heartbeat = now
            # Rewriting every table row is the expensive part; 4 Hz is ample
            # for 1 Hz statistics.
            if now - getattr(self, 'last_table_refresh', 0.) < .25:
                return
            self.last_table_refresh = now
            for (node, stage), (at, row) in self.latest.items():
                key = node + ':' + stage
                mean = row['mean_ms']
                values = (node, stage, self.caps.get(stage, '입력 연동'), f"{row['fps']:.1f}", f'{mean:.3f}' if mean > 0 else '--',
                          f"{row['p95_ms']:.3f}", f'{1000/mean:.1f}' if mean > 0 else '--',
                          'LIVE' if now-at < 2 else 'STALE')
                if self.table.exists(key):
                    self.table.item(key, values=values)
                else:
                    self.table.insert('', 'end', iid=key, values=values)


def main(args=None):
    rclpy.init(args=args)
    root = node = None
    try:
        try:
            import tkinter as tk
            root = tk.Tk()
            root.withdraw()
            node = RuntimeTools(root)
        except Exception as error:
            if root is not None:
                root.destroy()
                root = None
            message = f'Tools startup failed: {type(error).__name__}: {error}'
            if isinstance(error, ImportError) and 'tkinter' in str(error):
                message += '; install python3-tk'
            # Keep a ROS error responder alive so inspection buttons show the
            # actual startup failure instead of an unexplained unavailable state.
            node = Node('auto_runtime_tools')
            node.get_logger().error(message)
            def unavailable(request, response):
                response.success = False
                response.message = message
                return response
            for key in ('metrics', 'parameters'):
                node.create_service(Trigger, '/auto/runtime_tools/open_' + key, unavailable)
            rclpy.spin(node)
            return
        root.after(10, node.tick)
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if isinstance(node, RuntimeTools):
            try:
                node.set_metrics(False)
            except Exception:
                pass  # ROS may already be shutting down.
        if node is not None:
            node.destroy_node()
        if root is not None:
            root.destroy()
        if rclpy.ok():
            rclpy.shutdown()
