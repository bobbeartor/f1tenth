"""Standard-library HTTP + Server-Sent Events server for the remote preview.

GET /         preview page        GET /events  SSE stream of JSON frames
POST /start   POST /stop          POST /heartbeat (204)
The server never touches ROS; it talks to a backend with request_start/request_stop/heartbeat.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading

from .remote_link import LatestSlot


class RemoteServer:
    def __init__(self, bind, port, html_path, backend):
        self.backend = backend
        self.html_path = html_path
        self.slots = set()
        self.lock = threading.Lock()
        server = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def log_message(self, *args):
                pass

            def send_body(self, status, body, content_type):
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    with open(server.html_path, 'rb') as page:
                        self.send_body(200, page.read(), 'text/html; charset=utf-8')
                elif self.path == '/events':
                    server._stream(self)
                else:
                    self.send_body(404, b'not found', 'text/plain')

            def do_POST(self):
                length = int(self.headers.get('Content-Length') or 0)
                if length:
                    self.rfile.read(length)
                if self.path == '/heartbeat':
                    server.backend.heartbeat()
                    self.send_response(204)
                    self.send_header('Content-Length', '0')
                    self.end_headers()
                elif self.path in ('/start', '/stop'):
                    call = server.backend.request_start if self.path == '/start' else server.backend.request_stop
                    try:
                        ok, message = call()
                    except Exception as error:  # report, never crash the server thread
                        ok, message = False, f'{self.path[1:].upper()} failed: {error}'
                    body = json.dumps({'ok': bool(ok), 'message': str(message)}).encode()
                    self.send_body(200, body, 'application/json')
                else:
                    self.send_body(404, b'not found', 'text/plain')

        self.httpd = ThreadingHTTPServer((bind, port), Handler)
        self.httpd.daemon_threads = True
        self.port = self.httpd.server_address[1]
        self.thread = None

    def _stream(self, handler):
        slot = LatestSlot()
        with self.lock:
            self.slots.add(slot)
        try:
            handler.send_response(200)
            handler.send_header('Content-Type', 'text/event-stream')
            handler.send_header('Cache-Control', 'no-cache')
            handler.send_header('Connection', 'keep-alive')
            handler.end_headers()
            handler.wfile.write(b': connected\n\n')
            handler.wfile.flush()
            while True:
                item = slot.get(2.0)
                if slot.closed:
                    break
                handler.wfile.write(item if item is not None else b': ping\n\n')
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with self.lock:
                self.slots.discard(slot)
            handler.close_connection = True

    def start(self):
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

    def broadcast(self, frame):
        data = b'data: ' + json.dumps(frame, separators=(',', ':')).encode() + b'\n\n'
        with self.lock:
            slots = list(self.slots)
        for slot in slots:
            slot.put(data)

    def client_count(self):
        with self.lock:
            return len(self.slots)

    def close(self):
        with self.lock:
            slots = list(self.slots)
        for slot in slots:
            slot.close()
        self.httpd.shutdown()
        self.httpd.server_close()
