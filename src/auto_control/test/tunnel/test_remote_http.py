import http.client
import json
import os
import socket
import tempfile
import time
import unittest

from auto_control_tunnel.remote_http import RemoteServer


class FakeBackend:
    def __init__(self):
        self.calls = []

    def request_start(self):
        self.calls.append('start')
        return False, 'waiting_for_camera'

    def request_stop(self):
        self.calls.append('stop')
        return True, 'STOPPED'

    def heartbeat(self):
        self.calls.append('heartbeat')


def wait_for(predicate, timeout=1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.html = tempfile.NamedTemporaryFile('w', suffix='.html', delete=False)
        self.html.write('<canvas id="bev"></canvas>')
        self.html.close()
        self.backend = FakeBackend()
        self.server = RemoteServer('127.0.0.1', 0, self.html.name, self.backend)
        self.server.start()

    def tearDown(self):
        self.server.close()
        os.unlink(self.html.name)

    def request(self, method, path):
        conn = http.client.HTTPConnection('127.0.0.1', self.server.port, timeout=3)
        conn.request(method, path)
        response = conn.getresponse()
        return response.status, response.read()

    def open_stream(self):
        sock = socket.create_connection(('127.0.0.1', self.server.port), timeout=3)
        sock.sendall(b'GET /events HTTP/1.1\r\nHost: x\r\n\r\n')
        return sock

    def test_index_serves_html(self):
        status, body = self.request('GET', '/')
        self.assertEqual(status, 200)
        self.assertIn(b'<canvas', body)

    def test_events_stream_delivers_latest_frame(self):
        sock = self.open_stream()
        self.assertTrue(wait_for(lambda: self.server.client_count() == 1))
        self.server.broadcast({'seq': 1})
        self.server.broadcast({'seq': 2})
        data = b''
        while b'"seq":2' not in data:
            data += sock.recv(4096)
        sock.close()

    def test_slow_client_does_not_block_broadcast(self):
        sock = self.open_stream()  # never read
        self.assertTrue(wait_for(lambda: self.server.client_count() == 1))
        started = time.monotonic()
        for seq in range(1000):
            self.server.broadcast({'seq': seq, 'pad': 'x' * 500})
        self.assertLess(time.monotonic() - started, 0.5)
        sock.close()

    def test_start_stop_forward_to_backend(self):
        status, body = self.request('POST', '/start')
        self.assertEqual(json.loads(body), {'ok': False, 'message': 'waiting_for_camera'})
        self.request('POST', '/stop')
        self.assertEqual(self.backend.calls.count('stop'), 1)

    def test_heartbeat_and_client_count(self):
        status, _ = self.request('POST', '/heartbeat')
        self.assertEqual(status, 204)
        self.assertIn('heartbeat', self.backend.calls)
        first, second = self.open_stream(), self.open_stream()
        self.assertTrue(wait_for(lambda: self.server.client_count() == 2))
        first.close()
        self.server.broadcast({'seq': 1})  # write to closed socket drops the slot
        self.assertTrue(wait_for(lambda: (self.server.broadcast({'seq': 2}) or
                                          self.server.client_count() == 1), 3.0))
        second.close()

    def test_unknown_path_404(self):
        self.assertEqual(self.request('GET', '/nope')[0], 404)


if __name__ == '__main__':
    unittest.main()
