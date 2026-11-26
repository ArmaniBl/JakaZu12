"""Loopback HTTP API and static UI. ROS stays in this WSL process."""
import argparse
from functools import cache
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re
import secrets
import signal
import threading
import time
from urllib.parse import urlsplit, parse_qs
from xml.etree.ElementTree import ParseError, parse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from controller_manager_msgs.srv import ListControllers
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

from .runner import object_geometry, pose_transform
from .geometry import quaternion
from .profiles import load_profile
from .web_control import Controller, Settings
from .robot_view import ExecutionAnimation, JakaWorkcellModel, robot_snapshot
from .scan_progress import monitor_progress


def pending_request(future, requested, now, timeout=3.):
    """Drop cancelled/timed-out requests so service restarts cannot stall polling.

    rclpy.Future.done() is false for a cancelled future.
    """
    if future is None:return None
    if future.cancelled():return None
    if not future.done() and now-requested>timeout:
        future.cancel()
        return None
    return future


def table_perforations(layout, pose, size):
    table, grid = layout['table'], layout['grid']
    width, depth = (table[key] * .001 for key in ('width_mm', 'depth_mm'))
    pitch, diameter, offset_x, offset_y = (grid[key] * .001 for key in
                                           ('pitch_mm', 'diameter_mm', 'offset_x_mm', 'offset_y_mm'))
    columns = math.floor((width - diameter / 2 - offset_x) / pitch) + 1
    rows = math.floor((depth - diameter / 2 - offset_y) / pitch) + 1
    yaw = math.radians(layout['world_from_table']['yaw_deg'])
    c, s = math.cos(yaw), math.sin(yaw)
    to_local = pose[:2, :2].T @ [[c, -s], [s, c]]
    holes = []
    for row in range(rows):
        for col in range(columns):
            x, y = to_local @ [offset_x + col * pitch - width / 2, offset_y + row * pitch - depth / 2]
            if abs(x) + diameter / 2 <= size[0] / 2 and abs(y) + diameter / 2 <= size[1] / 2:
                holes.append([float(x), float(y)])
    return {'diameter': diameter, 'centers': holes}


def workcell_geometry(objects, frame, table_layout=None, *, simulation=False):
    """Return workpiece geometry and the visible table in the planning frame."""
    rendered = []
    for obj in objects:
        if obj.header.frame_id != frame:
            continue
        if obj.id == 'floor' and len(obj.primitives) == len(obj.primitive_poses) == 1:
            box = obj.primitives[0]
            if box.type != SolidPrimitive.BOX:
                continue
            shape = {'size': list(box.dimensions)}
            local_pose = obj.primitive_poses[0]
        elif obj.id == 'welding_workpiece' and len(obj.meshes) == len(obj.mesh_poses) == 1:
            mesh = obj.meshes[0]
            shape = {'vertices': [value for p in mesh.vertices for value in (p.x, p.y, p.z)],
                     'indices': [int(i) for triangle in mesh.triangles for i in triangle.vertex_indices]}
            local_pose = obj.mesh_poses[0]
        elif obj.id == 'welding_workpiece' and len(obj.primitives) == len(obj.primitive_poses) == 1:
            primitive = obj.primitives[0]
            kind = {SolidPrimitive.BOX: 'box', SolidPrimitive.CYLINDER: 'cylinder'}.get(primitive.type)
            if kind is None:
                continue
            shape = {'primitive': kind, 'size': list(primitive.dimensions)}
            local_pose = obj.primitive_poses[0]
        else:
            continue
        pose = pose_transform(obj.pose) @ pose_transform(local_pose)
        if obj.id == 'floor':
            if simulation:
                if table_layout is None:
                    continue
                # The Isaac collision floor is larger than the displayed workbench.
                top = pose[2, 3] + shape['size'][2] / 2
                table = table_layout['table']
                width, depth = (table[key] * .001 for key in ('width_mm', 'depth_mm'))
                transform = table_layout['world_from_table']
                yaw = math.radians(transform['yaw_deg'])
                c, s = math.cos(yaw), math.sin(yaw)
                pose = np.eye(4)
                pose[:2, :2] = [[c, -s], [s, c]]
                pose[:2, 3] = pose[:2, :2] @ [width / 2, depth / 2] + transform['translation_m'][:2]
                tabletop_thickness = .10
                pose[2, 3] = top - tabletop_thickness / 2
                shape['size'] = [width, depth, tabletop_thickness]
            if table_layout is not None:
                shape['perforations'] = table_perforations(table_layout, pose, shape['size'])
        rendered.append(dict(id=obj.id, position=pose[:3, 3].tolist(),
                             orientation=quaternion(pose[:3, :3]).tolist(), **shape))
    return rendered


def jaka_description(workspace):
    try:
        return Path(get_package_share_directory('jaka_description'))
    except PackageNotFoundError:
        return workspace / 'src/jaka_description'


def monitor(controller, stop):
    node = Node("welding_web_monitor")
    profile = controller.profile
    table_path = (Path(get_package_share_directory('ur10_isaac_moveit_config'))
                  / 'config/table_grid.json')
    table_layout = json.loads(table_path.read_text()) if table_path.is_file() else None
    trajectory = ActionClient(node, FollowJointTrajectory, profile.trajectory_action)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    raw_time, normalized_time = 0.0, 0.0
    robot_updated = 0.0
    animation = ExecutionAnimation(node, profile) if profile.real else None
    jaka_model = None
    sim_positions = None
    if not profile.real:
        try:
            jaka_model = JakaWorkcellModel(jaka_description(controller.workspace) / 'urdf/jaka_zu12.urdf')
        except (OSError, ParseError, KeyError, ValueError) as error:
            node.get_logger().warning(f'JAKA workcell model unavailable: {error}')
    def raw(message):
        nonlocal raw_time
        if profile.accepts(message) and all(math.isfinite(p) for p in message.position):
            raw_time = time.monotonic()
    def normalized(message):
        nonlocal normalized_time, sim_positions
        if profile.accepts(message) and all(math.isfinite(p) for p in message.position):
            normalized_time = time.monotonic()
            if jaka_model is not None:
                by_name = dict(zip(message.name, message.position))
                sim_positions = tuple(by_name[name] for name in profile.joints)
    subscriptions = [node.create_subscription(JointState, profile.source_topic, raw, qos_profile_sensor_data),
                     node.create_subscription(JointState, profile.joint_states, normalized, qos_profile_sensor_data)]
    scene = node.create_client(GetPlanningScene, "/get_planning_scene")
    controllers = node.create_client(ListControllers, "/controller_manager/list_controllers")
    scene_future = controller_future = None
    last_request, last_scene, last_controllers = 0.0, 0.0, 0.0
    scene_requested = controller_requested = 0.0
    scene_generation = 0
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if now-robot_updated >= .2:
                feedback_fresh = now-raw_time < 1.5 and now-normalized_time < 1.5
                ros_now = node.get_clock().now().nanoseconds / 1e9
                robot = robot_snapshot(tf_buffer, feedback_fresh, ros_now, profile.frames, profile.frame)
                workcell_pose = robot
                if not profile.real:
                    workcell_pose = dict(available=False, reason='Ожидаем обратную связь UR10 и TF робота')
                    if jaka_model is not None and sim_positions is not None:
                        source = robot_snapshot(tf_buffer, feedback_fresh, ros_now, ('base_link', 'tool0'),
                                                profile.frame, optional_frames=('welding_torch_link',))
                        if source['available']:
                            by_frame = {point['frame']: point for point in source['points']}
                            base = by_frame['base_link']
                            torch = by_frame.get('welding_torch_link', {})
                            points = jaka_model.points(sim_positions, base['position'], base['orientation'],
                                                       torch.get('orientation'))
                            workcell_pose = dict(available=True, points=points)
                animated_pose = None
                if animation is not None:
                    try:
                        animated_pose = animation.update(now)
                    except Exception as error:
                        node.get_logger().warning(f'Workcell animation unavailable: {error}', throttle_duration_sec=5.)
                with controller.lock:
                    controller.telemetry["robot"] = robot
                    controller.workcell_pose = workcell_pose
                    if workcell_pose['available']:
                        controller.last_workcell_pose = workcell_pose
                    controller.animation_pose = animated_pose
                robot_updated = now
            if scene_future is not None and scene_future.done():
                try:
                    objects, geometries = [], {}
                    collision_objects = scene_future.result().scene.world.collision_objects
                    visual_objects = workcell_geometry(collision_objects, profile.frame, table_layout,
                                                       simulation=not profile.real)
                    for obj in collision_objects:
                        if profile.real and not obj.meshes:
                            continue
                        try:
                            geometry = object_geometry(obj, profile.frame)
                            geometries[obj.id] = geometry
                            objects.append({"id": obj.id, 'shape': geometry.shape,
                                            'preset': obj.type.key if obj.type.key.startswith('cad:') else None,
                                            "dimensions": list(obj.primitives[0].dimensions) if obj.primitives else [],
                                            "signature": [c.vertices.tolist() for c in geometry.contours.values()]})
                        except ValueError:
                            continue
                    with controller.lock:
                        if scene_generation == controller.scene_generation:
                            controller.object_edges = geometries
                            controller.scene_received = True
                            if visual_objects != controller.workcell_objects:
                                controller.workcell_objects = visual_objects
                                controller.workcell_revision += 1
                            controller.telemetry["objects"] = objects
                            controller.telemetry['error'] = None
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
            scene_future=pending_request(scene_future,scene_requested,now)
            controller_future=pending_request(controller_future,controller_requested,now)
            if now-last_request >= .8:
                if scene_future is None and scene.service_is_ready():
                    with controller.lock:
                        scene_generation = controller.scene_generation
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
                    controller.last_telemetry = now
                last_request = now
    finally:
        node.destroy_node()


@cache
def simulation_torch_meshes(package):
    """Resolve the same visual meshes that MoveIt loads for the simulated torch."""
    model = parse(package / 'config/welding_torch_isaac.xacro')
    prefix = 'package://ur10_isaac_moveit_config/'
    meshes = {}
    for mesh in model.findall(".//link[@name='welding_torch_link']/visual/geometry/mesh"):
        filename = mesh.attrib['filename']
        if filename.startswith(prefix):
            path = package / filename.removeprefix(prefix)
            meshes[path.stem] = path
    return meshes


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
                if url.path == '/api/workcell':
                    revision = int(parse_qs(url.query).get('revision', ['-1'])[0])
                    with controller.lock:
                        fresh = time.monotonic() - controller.last_telemetry <= 4
                        live = fresh and controller.telemetry['moveit']
                        pose = controller.workcell_pose if fresh else {'available': False}
                        data = dict(live=live, real=controller.profile.real, robot=pose,
                                    revision=controller.workcell_revision,
                                    animation=controller.animation_pose if fresh else None)
                        if not pose['available']:
                            data['last_pose'] = controller.last_workcell_pose
                        if revision != controller.workcell_revision:
                            data['objects'] = controller.workcell_objects
                    return self.respond(200, data)
                mesh = re.fullmatch(r'/workcell-mesh/(base_link|[1-6]|visual_(?:[0-9]|10))\.stl', url.path)
                if mesh:
                    name = mesh[1]
                    if name.startswith('visual_'):
                        if controller.profile.real:
                            path = static.parent / 'meshes/torch' / f'{name}.stl'
                        else:
                            path = simulation_torch_meshes(static.parent).get(name)
                            if path is None:
                                return self.respond(404, {"error": "Модель детали горелки не найдена"})
                    else:
                        path = (jaka_description(controller.workspace) /
                                'meshes/jaka_zu12_meshes'/f'{name}.STL')
                    return self.respond(200, path.read_bytes(), 'model/stl')
                if url.path.startswith("/api/report/"):
                    return self.respond(200, controller.report(url.path.rsplit("/", 1)[1]))
                assets = {"/": ("index.html", "text/html; charset=utf-8"),
                          "/favicon.svg": ("favicon.svg", "image/svg+xml"),
                          "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                          "/workcell": ("workcell.html", "text/html; charset=utf-8"),
                          "/workcell.js": ("workcell.js", "text/javascript; charset=utf-8"),
                          "/workcell.css": ("workcell.css", "text/css; charset=utf-8"),
                          **{f"/workcell-vendor/{name}": (f"workcell-vendor/{name}", "text/javascript; charset=utf-8")
                             for name in ("three.module.js", "three.core.js", "OrbitControls.js", "RoomEnvironment.js")},
                          "/scan.js": ("scan.js", "text/javascript; charset=utf-8"),
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
    if profile.real:
        threading.Thread(target=monitor_progress, args=(controller, stop), daemon=True).start()
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
