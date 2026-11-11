"""Loopback HTTP API and static UI. ROS stays in this WSL process."""
import argparse
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import secrets
import signal
import threading
import time
from urllib.parse import urlsplit, parse_qs

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPlanningScene
from controller_manager_msgs.srv import ListControllers
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

from .runner import object_geometry
from .profiles import load_profile
from .web_control import Controller, Settings
from .robot_view import robot_snapshot


def monitor(controller, stop):
    node = Node("welding_web_monitor")
    profile = controller.profile
    trajectory = ActionClient(node, FollowJointTrajectory, profile.trajectory_action)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    raw_time, normalized_time = 0.0, 0.0
    robot_updated = 0.0
    def raw(message):
        nonlocal raw_time
        if profile.accepts(message) and all(math.isfinite(p) for p in message.position):
            raw_time = time.monotonic()
    def normalized(message):
        nonlocal normalized_time
        if profile.accepts(message) and all(math.isfinite(p) for p in message.position):
            normalized_time = time.monotonic()
    subscriptions = [node.create_subscription(JointState, profile.source_topic, raw, qos_profile_sensor_data),
                     node.create_subscription(JointState, profile.joint_states, normalized, qos_profile_sensor_data)]
    scene = node.create_client(GetPlanningScene, "/get_planning_scene")
    controllers = node.create_client(ListControllers, "/controller_manager/list_controllers")
    scene_future = controller_future = None
    last_request, last_scene, last_controllers = 0.0, 0.0, 0.0
    scene_requested = controller_requested = 0.0
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if now-robot_updated >= .2:
                robot = robot_snapshot(tf_buffer, now-raw_time < 1.5 and now-normalized_time < 1.5,
                                       node.get_clock().now().nanoseconds/1e9, profile.frames, profile.frame)
                with controller.lock:
                    controller.telemetry["robot"] = robot
                robot_updated = now
            if scene_future is not None and scene_future.done():
                try:
                    objects, geometries = [], {}
                    for obj in scene_future.result().scene.world.collision_objects:
                        try:
                            geometry = object_geometry(obj, profile.frame)
                            geometries[obj.id] = geometry
                            objects.append({"id": obj.id, 'shape': geometry.shape,
                                            "dimensions": list(obj.primitives[0].dimensions) if obj.primitives else [],
                                            "signature": [c.vertices.tolist() for c in geometry.contours.values()]})
                        except ValueError:
                            continue
                    with controller.lock:
                        controller.object_edges = geometries
                        controller.telemetry["objects"] = objects
                    last_scene = now
                except Exception as error:
                    with controller.lock:
                        controller.telemetry["error"] = str(error)
                scene_future = None
            if controller_future is not None and controller_future.done():
                try:
                    active = any(c.name == "ur10_controller" and c.state == "active"
                                 for c in controller_future.result().controller)
                    with controller.lock:
                        controller.telemetry["controller"] = active
                    last_controllers = now
                except Exception:
                    pass
                controller_future = None
            for future, requested in ((scene_future, scene_requested), (controller_future, controller_requested)):
                if future is not None and now-requested > 3:
                    future.cancel()
            if now-last_request >= .8:
                if scene_future is None and scene.service_is_ready():
                    scene_future = scene.call_async(GetPlanningScene.Request())
                    scene_requested = now
                if controller_future is None and controllers.service_is_ready():
                    controller_future = controllers.call_async(ListControllers.Request())
                    controller_requested = now
                isaac_publishers = node.get_publishers_info_by_topic(profile.source_topic)
                with controller.lock:
                    controller.telemetry.update(
                        isaac=(now-raw_time < 1.5 and bool(isaac_publishers)
                               and (all if profile.real else any)(profile.publisher_matches(p.node_name) for p in isaac_publishers)),
                        feedback=now-normalized_time < 1.5, moveit=now-last_scene < 3,
                    )
                    if now-last_controllers > 3:
                        controller.telemetry["controller"] = False
                    if profile.real:
                        controller.telemetry['controller'] = trajectory.server_is_ready()
                    if now-last_scene > 3:
                        controller.object_edges = {}
                        controller.telemetry["objects"] = []
                    controller.last_telemetry = now
                last_request = now
    finally:
        node.destroy_node()


def make_handler(controller, static, token, port):
    allowed_hosts = {f"localhost:{port}", f"127.0.0.1:{port}"}
    allowed_origins = {"http://"+host for host in allowed_hosts}
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def respond(self, code, payload, content_type="application/json; charset=utf-8"):
            try:
                self.write_response(code, payload, content_type)
            except (BrokenPipeError, ConnectionResetError):
                # Closing a tab does not cancel the owned ROS process.
                pass

        def write_response(self, code, payload, content_type):
            data = payload if isinstance(payload, bytes) else json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self'; script-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(data)

        def valid_host(self):
            return self.headers.get("Host") in allowed_hosts

        def do_GET(self):
            if not self.valid_host():
                return self.respond(403, {"error": "Недопустимый адрес панели"})
            url = urlsplit(self.path)
            try:
                if url.path == "/api/state":
                    since = int(parse_qs(url.query).get("since", ["0"])[0])
                    state = controller.status(since)
                    state["token"] = token
                    return self.respond(200, state)
                if url.path.startswith("/api/report/"):
                    return self.respond(200, controller.report(url.path.rsplit("/", 1)[1]))
                assets = {"/": ("index.html", "text/html; charset=utf-8"),
                          "/favicon.svg": ("favicon.svg", "image/svg+xml"),
                          "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                          "/style.css": ("style.css", "text/css; charset=utf-8")}
                if url.path in assets:
                    name, mime = assets[url.path]
                    return self.respond(200, (static/name).read_bytes(), mime)
                return self.respond(404, {"error": "Не найдено"})
            except (ValueError, OSError) as error:
                return self.respond(400, {"error": str(error)})

        def do_POST(self):
            if (not self.valid_host() or self.headers.get("Origin") not in allowed_origins
                    or not secrets.compare_digest(self.headers.get("X-Weld-Token", ""), token)):
                return self.respond(403, {"error": "Обновите страницу панели"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 16384 or self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                    raise ValueError("Ожидается JSON до 16 КБ")
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise ValueError("Ожидается объект JSON")
                if self.path == "/api/geometry":
                    return self.respond(200, controller.geometry(Settings.parse(data)))
                if self.path == '/api/workpiece':
                    return self.respond(200, controller.replace_workpiece(data))
                if self.path == "/api/start":
                    job = controller.start(data.get("mode"), Settings.parse(data.get("settings")))
                    return self.respond(200, job)
                if self.path == "/api/stop":
                    controller.stop()
                    return self.respond(200, {"ok": True})
                if self.path == "/api/scene":
                    if data.get("action") not in ("start", "stop") or not isinstance(data.get("rviz", True), bool):
                        raise ValueError("Некорректная команда сцены")
                    controller.scene(data["action"] == "start", data.get("rviz", True))
                    return self.respond(200, {"ok": True})
                return self.respond(404, {"error": "Не найдено"})
            except (ValueError, TypeError, OSError) as error:
                return self.respond(400, {"error": str(error)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description="Local welding web interface")
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument('--robot', choices=['isaac_ur10', 'jaka_real'], default='isaac_ur10')
    parser.add_argument('--robot-config')
    parser.add_argument('--return-pose')
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("Port must be in 1024..65535")
    executable = Path(get_package_prefix("ur10_isaac_moveit_config")) / "lib/ur10_isaac_moveit_config/weld.py"
    static = Path(get_package_share_directory("ur10_isaac_moveit_config")) / "web"
    profile = load_profile(args.robot, args.robot_config)
    controller = Controller(args.workspace, executable, profile, args.robot_config, args.return_pose)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(controller, static, secrets.token_hex(32), args.port))
    server.daemon_threads = True
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    thread = threading.Thread(target=monitor, args=(controller, stop), daemon=True)
    thread.start()
    server.timeout = .3
    def interrupt(*_args):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    print(f"Welding Studio: http://localhost:{args.port}", flush=True)
    try:
        while True:
            try:
                server.handle_request()
            except KeyboardInterrupt:
                try:
                    controller.close()
                except RuntimeError as error:
                    print(str(error), flush=True)
                    continue
                break
    finally:
        server.server_close()
        stop.set()
        thread.join(timeout=5)
        if rclpy.ok():
            rclpy.shutdown()
