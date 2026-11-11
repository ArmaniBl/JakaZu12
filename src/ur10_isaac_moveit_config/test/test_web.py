"""Validate browser inputs, local-origin checks and process cancellation."""
import http.client
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from http.server import ThreadingHTTPServer

from welding_sim.geometry import box_edges
from welding_sim.web_control import Controller, Settings, geometry_payload
from welding_sim.web_server import make_handler


class WebTests(unittest.TestCase):
    def test_settings_units_and_invalid_ranges(self):
        settings = Settings.parse({"selection": "edge", "begin": .2, "end": .8, "speed": .02})
        argv = settings.argv(Path('/some path/weld.py'), 'execute', '/tmp/reports')
        self.assertIn('/some path/weld.py', argv)
        self.assertIn('--edge', argv)
        self.assertNotIn('--face', argv)
        self.assertEqual(Settings.parse({"scaling": 1.0}).scaling, 1.0)
        for payload in ({"begin": .9, "end": .1}, {"speed": float('nan')},
                        {"offset": 0}, {"scaling": True}, {"scaling": 1.01},
                        {"scaling": 0}, {"edge": 'x; touch /tmp/oops'},
                        {"selection": 'shell'}, {"extra": 1}):
            with self.assertRaises(ValueError):
                Settings.parse(payload)

    def test_face_and_edge_geometry_lengths(self):
        edges = box_edges([.3, .3, .2], [-1, 0, .1])
        face = geometry_payload(edges, Settings())
        self.assertEqual(len(face['faces']), 6)
        self.assertAlmostEqual(face['length'], 1.2)
        edge = geometry_payload(edges, Settings(selection='edge', begin=.2, end=.8))
        self.assertAlmostEqual(edge['length'], .18)

    def test_cancel_child_and_reject_concurrent_execution(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            worker = root/'fake_weld.py'
            worker.write_text('''import argparse,json,signal,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output');p.add_argument('--mode');a,_=p.parse_known_args()
def done(*args):
 d=Path(a.output);d.mkdir(parents=True,exist_ok=True);(d/'result.json').write_text(json.dumps({'status':'interrupted'}));raise SystemExit(130)
signal.signal(signal.SIGINT,done)
print('ready',flush=True)
while True: time.sleep(.1)
''')
            controller = Controller(root, worker)
            controller.telemetry.update(moveit=True, isaac=True, feedback=True, controller=True)
            controller.last_telemetry = time.monotonic()
            controller.object_edges['welding_workpiece'] = box_edges([.3, .3, .2])
            try:
                controller.start('execute', Settings())
                deadline = time.monotonic()+5
                while not any(e['text']=='ready' for e in controller.logs) and time.monotonic()<deadline:
                    time.sleep(.02)
                with self.assertRaisesRegex(ValueError, 'Уже выполняется'):
                    controller.start('plan', Settings())
                controller.stop()
                controller.job_process.wait(timeout=5)
                deadline = time.monotonic()+3
                while controller.job['state']=='running' and time.monotonic()<deadline:
                    time.sleep(.02)
                self.assertEqual(controller.job['state'], 'cancelled')
                self.assertTrue(controller.job['report'])
                with self.assertRaises(ValueError):
                    controller.report('../../etc/passwd')
            finally:
                controller.close()

    def test_http_rejects_foreign_origin_and_missing_token(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = Controller(directory, Path(directory)/'unused.py')
            server = ThreadingHTTPServer(('127.0.0.1', 0), lambda *args: None)
            port = server.server_address[1]
            server.RequestHandlerClass = make_handler(controller, Path(directory), 'test-token', port)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            client = http.client.HTTPConnection('127.0.0.1', port)
            try:
                def post(origin, token):
                    client.request('POST', '/api/geometry', '{}', {'Content-Type':'application/json',
                                   'Origin':origin, 'X-Weld-Token':token})
                    response = client.getresponse()
                    code, payload = response.status, json.loads(response.read())
                    return code, payload
                self.assertEqual(post('https://other.example', 'test-token')[0], 403)
                self.assertEqual(post(f'http://127.0.0.1:{port}', '')[0], 403)
                code, payload = post(f'http://127.0.0.1:{port}', 'test-token')
                self.assertEqual(code, 200)
                self.assertTrue(payload['demo'])
            finally:
                client.close()
                server.shutdown()
                server.server_close()
                thread.join(timeout=3)


if __name__ == '__main__':
    unittest.main()
