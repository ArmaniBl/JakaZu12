"""Select workpiece contours and plan/execute them with a robot-specific profile."""
import argparse
from collections import deque
from copy import deepcopy
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Point, Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, DisplayTrajectory, JointConstraint, RobotState, PlanningSceneComponents
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPlanningScene, GetPositionFK, GetPositionIK
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import quaternion, rotation
from .return_pose import load_return_pose
from .profiles import ISAAC, load_profile
from .orientation import tilted_orientation, candidates
from .workpiece_model import PREVIEW_TYPE
from .geometry import box_geometry, cylinder_geometry, mesh_geometry


def pose_message(position, orientation):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = map(float, orientation)
    return pose


def pose_transform(pose):
    matrix = np.eye(4)
    q = pose.orientation
    matrix[:3, :3] = rotation([q.x, q.y, q.z, q.w])
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def object_geometry(obj, frame='world'):
    if obj.id == 'floor' or obj.id.startswith('workspace_'):
        raise ValueError('floor is a collision obstacle, not a welding workpiece')
    if obj.header.frame_id != frame:
        raise ValueError(f'Workpiece must be expressed in the {frame} frame')
    if obj.planes or len(obj.primitives)+len(obj.meshes) != 1:
        raise ValueError('Use one BOX, CYLINDER or triangle mesh per workpiece')
    poses = obj.primitive_poses if obj.primitives else obj.mesh_poses
    if len(poses) != 1:
        raise ValueError('Workpiece requires exactly one geometry pose')
    transform = pose_transform(obj.pose) @ pose_transform(poses[0])
    center, q = transform[:3, 3], quaternion(transform[:3, :3])
    if obj.meshes:
        mesh = obj.meshes[0]
        geometry=mesh_geometry([[v.x, v.y, v.z] for v in mesh.vertices],
                              [list(t.vertex_indices) for t in mesh.triangles], center, q)
        from .workpiece_model import apply_model_seams
        return apply_model_seams(obj,geometry,center,q)
    primitive = obj.primitives[0]
    builders = {SolidPrimitive.BOX: box_geometry, SolidPrimitive.CYLINDER: cylinder_geometry}
    if primitive.type not in builders:
        raise ValueError('Supported shapes: BOX, CYLINDER, planar triangle meshes')
    return builders[primitive.type](primitive.dimensions, center, q)


def object_edges(obj):
    return object_geometry(obj).edges


def profile_of(node):
    # Default keeps old embedders/test stubs compatible.
    return getattr(node, 'profile', ISAAC)


class WeldingRunner(Node):
    def __init__(self, args):
        super().__init__("welding_simulator")
        self.args = args
        self.profile = load_profile(args.robot, args.robot_config)
        self.latest_state = None
        self.received_at = 0.0
        self.subscription = self.create_subscription(
            JointState, self.profile.joint_states, self.receive_state, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, "/welding/markers", qos)
        self.display = self.create_publisher(DisplayTrajectory, "/display_planned_path", qos)
        self.service_clients = {}
        self.active_goal = None
        self.cartesian_targets = {}
        self.joint_targets = {}
        self.return_pose = (load_return_pose(args.return_pose, self.profile.joints)
                            if args.return_pose or not self.profile.real else None)
        self.report = {"object": args.object, "mode": args.mode, "status": "started", "stages": []}
        self.report['robot_profile'] = self.profile.payload()

    def receive_state(self, message):
        if (self.profile.accepts(message)
                and all(math.isfinite(p) for p in message.position)):
            self.latest_state = message
            self.received_at = time.monotonic()

    def call(self, service, name, request, timeout=30.0):
        if name not in self.service_clients:
            self.service_clients[name] = self.create_client(service, name)
        client = self.service_clients[name]
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"Service unavailable: {name}. Start the selected robot's MoveIt launch")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            raise RuntimeError(f"Service timed out: {name}")
        return future.result()

    def read_object(self):
        deadline = time.monotonic() + 20
        while True:
            scene = self.call(GetPlanningScene, "/get_planning_scene", GetPlanningScene.Request()).scene
            floor=next((o for o in scene.world.collision_objects if o.id=='floor'),None)
            if not self.profile.real and self.args.mode!='preview' and floor is None:
                if time.monotonic()>=deadline:
                    raise RuntimeError('Simulation collision floor is absent. Restart run_sim.sh or run add_floor.py before planning')
                rclpy.spin_once(self,timeout_sec=.2)
                continue
            for obj in scene.world.collision_objects:
                if obj.id == self.args.object:
                    if self.args.mode=='execute' and obj.type.key==PREVIEW_TYPE and not getattr(self.args,'use_scene_workpiece',False):
                        raise RuntimeError('CAD placement is a scanner preview, not a validated robot registration. Calibrate, detect and apply the verified scan before execution.')
                    self.workpiece = deepcopy(obj)
                    self.initial_scene = scene
                    self.planning_floor=deepcopy(floor)
                    self.geometry = object_geometry(obj, self.profile.frame)
                    return self.geometry.edges
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Object '{self.args.object}' is absent from the selected MoveIt scene")
            rclpy.spin_once(self, timeout_sec=0.2)

    def fresh_state(self):
        requested = time.monotonic()
        deadline = requested + 5
        # Wait for a new feedback callback, including after an action completes.
        # A cached sample can still describe motion during the approach.
        while self.latest_state is None or self.received_at <= requested:
            if time.monotonic() > deadline:
                raise RuntimeError(f"No fresh valid joint states on {self.profile.joint_states}; expected {self.profile.joints}")
            rclpy.spin_once(self, timeout_sec=0.1)
        state = deepcopy(self.initial_scene.robot_state)
        state.joint_state = deepcopy(self.latest_state)
        return state

    def settled_state(self, timeout=6.0, window=0.3, tolerance=0.002):
        deadline = time.monotonic() + timeout
        samples = deque()
        while True:
            state = self.fresh_state()
            now = time.monotonic()
            values = dict(zip(state.joint_state.name, state.joint_state.position))
            if samples and values.keys() != samples[-1][1].keys():
                samples.clear()
            samples.append((now, values))
            while len(samples) > 2 and samples[1][0] <= now-window:
                samples.popleft()
            if now-samples[0][0] >= window:
                span = max(max(v[joint] for _, v in samples)-min(v[joint] for _, v in samples)
                           for joint in values)
                if span <= tolerance:
                    return state
            if now >= deadline:
                raise RuntimeError("Robot joints did not settle before execution; check drives and tracking")

    @staticmethod
    def start_errors(state, trajectory):
        path = trajectory.joint_trajectory
        if not path.points or len(path.joint_names) != len(path.points[0].positions):
            raise RuntimeError("Trajectory has no valid start point")
        actual = dict(zip(state.joint_state.name, state.joint_state.position))
        errors = {}
        for name, target in zip(path.joint_names, path.points[0].positions):
            if name not in actual or not math.isfinite(target) or not math.isfinite(actual[name]):
                raise RuntimeError(f"Missing or invalid joint state: {name}")
            errors[name] = abs(target-actual[name])
        if not errors:
            raise RuntimeError("Trajectory has no joints")
        return errors

    def prepare_execution(self, name, trajectory):
        joint_targets = getattr(self, "joint_targets", {})
        replannable = name in self.cartesian_targets or name in joint_targets
        for attempt in range(1, 4):
            actual = self.settled_state()
            if name in self.cartesian_targets:
                trajectory = self.cartesian(actual, self.cartesian_targets[name])
            elif name in joint_targets:
                trajectory = self.joint_motion(actual, joint_targets[name])
            if replannable:
                replanned = self.report.setdefault("feedback_replanned_stages", [])
                if name not in replanned:
                    replanned.append(name)
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ROBOT_STATE
            moveit_state = self.call(GetPlanningScene, "/get_planning_scene", request).scene.robot_state
            actual = self.fresh_state()
            errors = self.start_errors(actual, trajectory)
            moveit_errors = self.start_errors(moveit_state, trajectory)
            worst = max(errors, key=lambda joint: max(errors[joint], moveit_errors[joint]))
            deviation = max(errors[worst], moveit_errors[worst])
            self.report.setdefault("execution_start_checks", []).append({
                "stage": name, "attempt": attempt, "joint_names": list(trajectory.joint_trajectory.joint_names),
                "trajectory_start": list(trajectory.joint_trajectory.points[0].positions),
                "feedback": dict(zip(actual.joint_state.name, actual.joint_state.position)),
                "moveit_state": dict(zip(moveit_state.joint_state.name, moveit_state.joint_state.position)),
                "feedback_errors": errors, "moveit_errors": moveit_errors,
                "max_error_rad": deviation, "worst_joint": worst})
            if deviation <= 0.005:
                return trajectory
            if not replannable or attempt == 3:
                raise RuntimeError(f"Robot state changed before {name}: {worst} start error {deviation:.6f} rad. "
                                   "Trajectory was not sent; inspect execution_start_checks in the report")
            self.get_logger().warning(f"Robot state changed during path preparation ({worst}: {deviation:.6f} rad); waiting and replanning")

    def publish_markers(self, edges, selected, paths):
        messages = [Marker(action=Marker.DELETEALL)]
        selected_ids = {edge.id for edge in selected}
        for edge in edges:
            marker = Marker()
            marker.header.frame_id = self.profile.frame
            marker.ns = "edges"
            marker.id = len(messages)
            marker.type = Marker.LINE_STRIP
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.006 if edge.id in selected_ids else 0.002
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                (0.1, 1.0, 0.2, 1.0) if edge.id in selected_ids else (0.6, 0.6, 0.6, 0.6))
            values = edge.vertices if hasattr(edge, 'vertices') else (edge.start, edge.end)
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in values]
            messages.append(marker)
            label = deepcopy(marker)
            label.ns = "labels"
            label.type = Marker.TEXT_VIEW_FACING
            label.points = []
            label.scale.z = 0.022
            label.text = edge.id
            mid = (edge.start + edge.end)/2 + edge.outward * 0.025
            label.pose = pose_message(mid, [0, 0, 0, 1])
            messages.append(label)
        for edge, seam, points, q in paths:
            for namespace, values, color in (
                    ("selected_seam", seam, (1.0, 0.1, 0.1)),
                    ("tool_path", points, (0.0, 0.8, 1.0))):
                marker = Marker()
                marker.header.frame_id = self.profile.frame
                marker.ns, marker.id = namespace, len(messages)
                marker.type = Marker.LINE_STRIP
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.004
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.0
                marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in values]
                messages.append(marker)
        self.markers.publish(MarkerArray(markers=messages))

    def approach(self, state, position, orientation, seed=None):
        ik = GetPositionIK.Request()
        ik.ik_request.group_name = profile_of(self).group
        ik.ik_request.robot_state = state if seed is None else seed
        ik.ik_request.ik_link_name = profile_of(self).tip
        ik.ik_request.pose_stamped.header.frame_id = profile_of(self).frame
        ik.ik_request.pose_stamped.pose = pose_message(position, orientation)
        ik.ik_request.avoid_collisions = True
        if getattr(self,'orientation_deadline',None) is not None:
            ik.ik_request.timeout.nanosec = 250_000_000
        else:
            ik.ik_request.timeout.sec = 2
        solution = self.call(GetPositionIK, "/compute_ik", ik)
        if solution.error_code.val != 1:
            raise RuntimeError(f"Approach IK failed ({solution.error_code.val}); change edge, offset or --roll")
        return self.joint_motion(state, dict(zip(solution.solution.joint_state.name,
                                                solution.solution.joint_state.position)))

    def joint_motion(self, state, targets):
        request = GetMotionPlan.Request()
        req = request.motion_plan_request
        req.group_name, req.pipeline_id = profile_of(self).group, "ompl"
        req.start_state = state
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = self.args.scaling
        req.max_acceleration_scaling_factor = self.args.scaling
        constraint = Constraints()
        missing = set(profile_of(self).joints)-set(targets)
        if missing:
            raise RuntimeError(f'Joint goal is missing configured joints: {sorted(missing)}')
        for name, value in targets.items():
            if name in profile_of(self).joints:
                if name not in self.latest_state.name or not math.isfinite(value):
                    raise RuntimeError(f'Missing or invalid joint target/feedback: {name}')
                constraint.joint_constraints.append(JointConstraint(
                    joint_name=name, position=value, tolerance_above=0.001,
                    tolerance_below=0.001, weight=1.0))
        req.goal_constraints = [constraint]
        result = self.call(GetMotionPlan, "/plan_kinematic_path", request).motion_plan_response
        if result.error_code.val != 1:
            raise RuntimeError(f"Collision-free joint-motion planning failed ({result.error_code.val})")
        return result.trajectory

    def cartesian(self, state, waypoints):
        request = GetCartesianPath.Request()
        request.header.frame_id = profile_of(self).frame
        request.group_name, request.link_name = profile_of(self).group, profile_of(self).tip
        request.start_state = state
        request.waypoints = waypoints
        request.max_step = self.args.step
        request.jump_threshold = 5.0
        request.revolute_jump_threshold = 0.3
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.args.scaling
        request.max_acceleration_scaling_factor = self.args.scaling
        if hasattr(request, "cartesian_speed_limited_link"):
            request.cartesian_speed_limited_link = profile_of(self).tip
            request.max_cartesian_speed = self.args.speed
        result = self.call(GetCartesianPath, "/compute_cartesian_path", request)
        self.report["last_cartesian_fraction"] = result.fraction
        if result.error_code.val != 1 or result.fraction < 1.0 - 1e-9:
            raise RuntimeError(f"Incomplete Cartesian path: {result.fraction:.1%}, code {result.error_code.val}. Nothing executed")
        self.limit_cartesian_speed(state, result.solution)
        return result.solution

    def limit_cartesian_speed(self, state, trajectory):
        """Enforce the requested sampled TCP speed even on older MoveIt servers."""
        previous = None
        factor = 1.0
        for point in trajectory.joint_trajectory.points:
            request = GetPositionFK.Request()
            request.header.frame_id = profile_of(self).frame
            request.fk_link_names = [profile_of(self).tip]
            request.robot_state = deepcopy(state)
            values = dict(zip(state.joint_state.name, state.joint_state.position))
            values.update(zip(trajectory.joint_trajectory.joint_names, point.positions))
            request.robot_state.joint_state = JointState(name=list(values), position=list(values.values()))
            result = self.call(GetPositionFK, "/compute_fk", request)
            if result.error_code.val != 1 or not result.pose_stamped:
                raise RuntimeError("Cannot verify TCP speed: forward kinematics failed")
            position = result.pose_stamped[0].pose.position
            position = np.array([position.x, position.y, position.z])
            stamp = point.time_from_start.sec + point.time_from_start.nanosec/1e9
            if previous is not None:
                previous_position, previous_stamp = previous
                if stamp <= previous_stamp:
                    raise RuntimeError("MoveIt returned non-increasing trajectory times")
                speed = np.linalg.norm(position-previous_position) / (stamp-previous_stamp)
                factor = max(factor, float(speed/self.args.speed))
            previous = position, stamp
        if factor > 1.0:
            factor *= 1.01
            for point in trajectory.joint_trajectory.points:
                seconds = (point.time_from_start.sec + point.time_from_start.nanosec/1e9) * factor
                nanoseconds = round(seconds*1e9)
                point.time_from_start = Duration(sec=nanoseconds//1_000_000_000, nanosec=nanoseconds%1_000_000_000)
                point.velocities = [v/factor for v in point.velocities]
                point.accelerations = [a/(factor*factor) for a in point.accelerations]
        self.report["tcp_speed_time_scale"] = factor

    @staticmethod
    def end_state(state, trajectory):
        result = deepcopy(state)
        joint_path = trajectory.joint_trajectory
        if not joint_path.points:
            raise RuntimeError("MoveIt returned an empty trajectory")
        values = dict(zip(result.joint_state.name, result.joint_state.position))
        values.update(zip(joint_path.joint_names, joint_path.points[-1].positions))
        result.joint_state = JointState(name=list(values), position=list(values.values()))
        return result

    def plan(self, paths):
        if profile_of(self).real:
            return self.plan_best(paths)
        automatic=getattr(self.args,'orientation_mode','fixed')=='auto'
        tilt=getattr(self.args,'tilt',0.)
        direction=getattr(self.args,'tilt_direction',0.)
        options=candidates(tilt,direction,self.args.max_tilt) if automatic else [(tilt,direction,0.)]
        self.orientation_deadline=time.monotonic()+getattr(self.args,'orientation_timeout',120.) if automatic else None
        attempts=[]
        try:
            for inclination,heading,roll in options:
                if self.orientation_deadline is not None and time.monotonic()>=self.orientation_deadline:break
                candidate=[(edge,seam,points,tilted_orientation(q,inclination,heading,roll))
                           for edge,seam,points,q in paths]
                self.cartesian_targets={};self.joint_targets={}
                entry={'tilt_deg':inclination,'direction_deg':heading,'roll_delta_deg':roll}
                if automatic:self.get_logger().info(f'Orientation search: tilt={inclination:g}, direction={heading:g}, roll_delta={roll:g}')
                try:
                    result=WeldingRunner.plan_fixed(self,candidate)
                    entry['status']='planned';attempts.append(entry)
                    self.report['orientation_attempts']=attempts
                    self.report['chosen_orientation']=entry.copy()
                    self.planned_paths=candidate
                    for i,(_,_,_,q) in enumerate(candidate):
                        if i<len(self.report.get('selection',[])):
                            self.report['selection'][i]['orientation_xyzw']=q.tolist()
                    return result
                except RuntimeError as error:
                    entry.update(status='failed',error=str(error));attempts.append(entry)
                    self.report['orientation_attempts']=attempts
                    if not automatic:raise
            self.report['orientation_attempts']=attempts
            raise RuntimeError(f'No complete collision-free route in {len(attempts)} orientation attempts. '
                               'Inspect orientation_attempts; change tilt limit, workpiece placement or selected seam.')
        finally:
            self.orientation_deadline=None

    def plan_best(self,paths):
        from types import SimpleNamespace
        from .table_limits import require_walls
        from .path_quality import trajectory_quality
        require_walls(self.initial_scene)
        automatic=self.args.orientation_mode=='auto'
        options=candidates(self.args.tilt,self.args.tilt_direction,self.args.max_tilt) if automatic else [(self.args.tilt,self.args.tilt_direction,0.)]*4
        start=self.settled_state()
        self.optimization_start=deepcopy(start)
        values=dict(zip(start.joint_state.name,start.joint_state.position))
        original=deepcopy(self.report);publisher=self.display
        self.display=SimpleNamespace(publish=lambda _:None)
        self.orientation_deadline=time.monotonic()+self.args.orientation_timeout
        attempts=[];best=None;successes=0
        try:
            for tilt,direction,roll in options:
                if time.monotonic()>=self.orientation_deadline or successes>=12:break
                self.report=deepcopy(original);self.cartesian_targets={};self.joint_targets={}
                candidate=[(edge,seam,points,tilted_orientation(q,tilt,direction,roll)) for edge,seam,points,q in paths]
                entry=dict(tilt_deg=tilt,direction_deg=direction,roll_delta_deg=roll)
                try:
                    stages=self.plan_fixed(candidate)
                    quality=trajectory_quality(stages,values,self.profile.joints)
                    successes+=1;entry.update(status='planned',quality=quality)
                    if best is None or quality['cost']<best['quality']['cost']:
                        best=dict(stages=stages,quality=quality,report=deepcopy(self.report),paths=candidate,
                            cartesian=deepcopy(self.cartesian_targets),joint=deepcopy(self.joint_targets),orientation=entry.copy())
                except (RuntimeError,ValueError) as error:entry.update(status='failed',error=str(error))
                attempts.append(entry)
                self.get_logger().info(f'Route comparison: {len(attempts)} attempts, {successes} complete candidates')
        finally:
            self.display=publisher;self.optimization_start=None;self.orientation_deadline=None
        if best is None:
            self.report=original;self.report['orientation_attempts']=attempts
            raise RuntimeError('No complete collision-free route inside table limits')
        actual=self.fresh_state();current=dict(zip(actual.joint_state.name,actual.joint_state.position))
        if max(abs(current[j]-values[j]) for j in self.profile.joints)>.005:
            raise RuntimeError('Robot moved during route comparison; plan again')
        scene=self.call(GetPlanningScene,'/get_planning_scene',GetPlanningScene.Request()).scene
        if (sorted(scene.world.collision_objects,key=lambda o:o.id)!=sorted(self.initial_scene.world.collision_objects,key=lambda o:o.id)
                or scene.robot_state.attached_collision_objects!=self.initial_scene.robot_state.attached_collision_objects):
            raise RuntimeError('Collision scene changed during route comparison; plan again')
        self.report=best['report'];self.cartesian_targets=best['cartesian'];self.joint_targets=best['joint'];self.planned_paths=best['paths']
        self.report.update(orientation_attempts=attempts,chosen_orientation=best['orientation'],optimization=dict(
            quality=best['quality'],complete_candidates=successes,scope='best sampled complete route; no global optimality guarantee'))
        for i,(_,_,_,q) in enumerate(best['paths']):
            if i<len(self.report.get('selection',[])):self.report['selection'][i]['orientation_xyzw']=q.tolist()
        publisher.publish(DisplayTrajectory(model_id=self.profile.model,trajectory_start=start,trajectory=[t for _,t in best['stages']]))
        return best['stages']

    def plan_fixed(self, paths):
        start = deepcopy(self.optimization_start) if getattr(self,'optimization_start',None) is not None else self.fresh_state()
        if self.return_pose is None:
            positions = dict(zip(start.joint_state.name, start.joint_state.position))
            self.return_pose = {'units': 'rad', 'source': 'job_start',
                                'joint_positions': {j: positions[j] for j in profile_of(self).joints}}
        state = deepcopy(start)
        stages = []
        for path_index, (edge, seam, points, orientation) in enumerate(paths):
            self.get_logger().info(f"Planning {edge.id}")
            pre = points[0] + self.args.clearance * edge.outward
            waypoints = [pose_message(p, orientation) for p in points]
            final_lift = profile_of(self).real and path_index == len(paths)-1
            if not final_lift:
                waypoints.append(pose_message(points[-1] + self.args.clearance * edge.outward, orientation))
            # A reachable start pose can have several IK branches. Search for
            # one that supports the entire contour before moving the robot.
            seeds = [state]
            for elbow in (() if profile_of(self).real else (-1.5, 1.5)):
                for wrist in (-1.57, 1.57):
                    seed = deepcopy(state)
                    values = dict(zip(seed.joint_state.name, seed.joint_state.position))
                    values.update(zip(profile_of(self).joints, (-3.14, -1.57, elbow, -1.57, wrist, 0.0)))
                    seed.joint_state = JointState(name=list(values), position=list(values.values()))
                    seeds.append(seed)
            last_error = None
            if getattr(self,'orientation_deadline',None) is not None:seeds=seeds[:2]
            for attempt, seed in enumerate(seeds, 1):
                if getattr(self,'orientation_deadline',None) is not None and time.monotonic()>=self.orientation_deadline:
                    raise RuntimeError('Orientation search time budget exhausted')
                try:
                    approach = self.approach(state, pre, orientation, seed)
                    approached = self.end_state(state, approach)
                    weld = self.cartesian(approached, waypoints)
                    break
                except RuntimeError as error:
                    last_error = error
                    self.get_logger().warning(f"IK branch {attempt}/{len(seeds)}: {error}")
            else:
                raise RuntimeError(f"No complete approach and contour found: {last_error}")
            stages.append((f"{edge.id}:approach", approach))
            state = approached
            stage_name = f"{edge.id}:pass_and_retract"
            self.cartesian_targets[stage_name] = waypoints
            stages.append((stage_name, weld))
            state = self.end_state(state, weld)
        if profile_of(self).real:
            # End above the last welding point, with unchanged TCP orientation.
            # This stage must be reachable before any job motion is allowed.
            self.get_logger().info("Planning vertical lift of 0.25 m after welding")
            target = points[-1].copy()
            target[2] += 0.25
            lift_waypoints = [pose_message(target, orientation)]
            try:
                finish = self.cartesian(state, lift_waypoints)
            except RuntimeError as error:
                raise RuntimeError(f'Completion lift failed: {error}') from error
            self.cartesian_targets["workpiece:lift"] = lift_waypoints
            stages.append(("workpiece:lift", finish))
            self.report["completion"] = {"mode": "vertical_lift", "height_m": 0.25,
                                         "target_xyz": target.tolist(),
                                         "orientation_xyzw": orientation.tolist()}
        else:
            self.get_logger().info("Planning return to the saved joint configuration")
            targets = dict(self.return_pose['joint_positions'])
            finish = self.joint_motion(state, targets)
            self.joint_targets["workpiece:return"] = targets
            stages.append(("workpiece:return", finish))
            self.report["return_pose"] = deepcopy(self.return_pose)
        self.report["stages"] = [{"name": name, "points": len(t.joint_trajectory.points),
                                  "duration_sec": t.joint_trajectory.points[-1].time_from_start.sec
                                  + t.joint_trajectory.points[-1].time_from_start.nanosec/1e9}
                                 for name, t in stages]
        self.display.publish(DisplayTrajectory(model_id=profile_of(self).model, trajectory_start=start,
                                               trajectory=[t for _, t in stages]))
        self.report["status"] = "planned"
        return stages

    def wait_for_isaac_publisher(self, timeout=10.0):
        profile = profile_of(self)
        deadline = time.monotonic() + timeout
        waiting = False
        while True:
            publishers = self.get_publishers_info_by_topic(profile.source_topic)
            names = [p.node_name for p in publishers]
            self.report["isaac_joint_state_publishers"] = names
            if any(profile.publisher_matches(name) for name in names):
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    f"Isaac OmniGraph publisher on /joint_states was not discovered within {timeout:g}s. "
                    f"Discovered publishers: {', '.join(names) or 'none'}. "
                    "Check Play in Isaac Sim, the joint-state graph and Windows/WSL DDS connection.")
            if not waiting:
                self.get_logger().info("Waiting for discovery of the Isaac /joint_states publisher...")
                waiting = True
            rclpy.spin_once(self, timeout_sec=min(0.2, remaining))

    def execute(self, stages):
        if self.profile.real:
            self.wait_for_robot_publisher()
        else:
            self.wait_for_isaac_publisher()
        action = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        if not action.wait_for_server(timeout_sec=10):
            raise RuntimeError("MoveIt execute action unavailable")
        for name, trajectory in stages:
            if getattr(self.args, 'scan_report', None):
                # Use the installed bridge and its separate numerical environment.
                from ament_index_python.packages import get_package_prefix
                bridge = Path(get_package_prefix('ur10_isaac_moveit_config'))/'lib/ur10_isaac_moveit_config/scan_job.py'
                python = Path(self.args.scan_report).resolve()
                workspace = next((p for p in python.parents if (p/'.venv-rangevision/bin/python').is_file()), None)
                if workspace is None:
                    raise RuntimeError('Scan processing environment unavailable')
                check = subprocess.run([str(workspace/'.venv-rangevision/bin/python'), str(bridge), 'check',
                    '--report', self.args.scan_report, '--calibration', self.args.scan_calibration,
                    '--scanner-url', self.args.scanner_url], capture_output=True, text=True, timeout=30)
                if check.returncode:
                    raise RuntimeError('Scan approval invalid before execution: '+check.stderr[-1500:])
            scene = self.call(GetPlanningScene, "/get_planning_scene", GetPlanningScene.Request()).scene
            current = next((o for o in scene.world.collision_objects if o.id == self.args.object), None)
            if current != self.workpiece:
                raise RuntimeError("Workpiece changed since planning; replan before execution")
            if getattr(self,'planning_floor',None) is not None:
                floor=next((o for o in scene.world.collision_objects if o.id=='floor'),None)
                if floor!=self.planning_floor:
                    raise RuntimeError('Collision floor changed or was removed since planning; replan before execution')
            trajectory = self.prepare_execution(name, trajectory)
            self.get_logger().info(f"Executing {name}")
            future = action.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory))
            rclpy.spin_until_future_complete(self, future, timeout_sec=10)
            if not future.done() or not future.result().accepted:
                raise RuntimeError(f"Execution goal rejected or timed out: {name}")
            handle = future.result()
            self.active_goal = handle
            result = handle.get_result_async()
            duration = trajectory.joint_trajectory.points[-1].time_from_start
            rclpy.spin_until_future_complete(self, result, timeout_sec=duration.sec + duration.nanosec/1e9 + 30)
            if not result.done():
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel, timeout_sec=5)
                raise RuntimeError(f"Execution timed out and cancellation requested: {name}")
            if result.result().result.error_code.val != 1:
                raise RuntimeError(f"Execution failed: {name}, code {result.result().result.error_code.val}")
            if name == "workpiece:return":
                self.verify_return_pose()
            self.report.setdefault("completed_stages", []).append(name)
            self.active_goal = None
        self.report["status"] = "executed"

    def verify_return_pose(self):
        actual = self.settled_state()
        values = dict(zip(actual.joint_state.name, actual.joint_state.position))
        targets = self.return_pose['joint_positions']
        if any(name not in values for name in targets):
            raise RuntimeError("Cannot verify saved return pose: missing joint feedback")
        errors = {name: abs(values[name]-target) for name, target in targets.items()}
        self.report['return_pose_actual'] = {name: values[name] for name in targets}
        self.report['return_pose_errors_rad'] = errors
        worst = max(errors, key=errors.get)
        if errors[worst] > .01:
            raise RuntimeError(f"Saved return pose was not reached: {worst} error {errors[worst]:.6f} rad")

    def wait_for_robot_publisher(self, timeout=10.):
        deadline = time.monotonic()+timeout
        while True:
            names = [p.node_name for p in self.get_publishers_info_by_topic(self.profile.source_topic)]
            self.report['robot_joint_state_publishers'] = names
            if names and all(self.profile.publisher_matches(n) for n in names):
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Expected only {self.profile.publisher} on {self.profile.source_topic}; got {names}')
            rclpy.spin_once(self, timeout_sec=.1)


def arguments():
    parser = argparse.ArgumentParser(description="Welding contour planning for Isaac UR10 or physical JAKA")
    parser.add_argument('--robot', choices=['isaac_ur10', 'jaka_real'], default='isaac_ur10')
    parser.add_argument('--robot-config', help='JSON overrides for robot names, links and topics')
    parser.add_argument('--return-pose', help='JSON joint return configuration for the selected robot')
    parser.add_argument("--object", default="welding_workpiece")
    parser.add_argument("--list", action="store_true", help="List detected edges and exit")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--edge", action="append", default=[], help="Alternative: select individual edge IDs")
    selection.add_argument("--face", help="Face ID from --list (default z_plus)")
    parser.add_argument("--begin", type=float, default=0.0, help="Beginning fraction of the contour/edge")
    parser.add_argument("--end", type=float, default=1.0, help="End fraction of the contour/edge")
    parser.add_argument("--offset", type=float, default=0.03, help="tool0 standoff along face normal (edge bisector in edge mode), m; default 0.03")
    parser.add_argument("--clearance", type=float, default=0.05, help="Additional approach/retract clearance, m")
    parser.add_argument("--step", type=float, default=0.005, help="Cartesian interpolation step, m")
    parser.add_argument("--roll", type=float, default=0.0, help="Tool rotation about its Z axis, degrees")
    parser.add_argument('--orientation-mode', choices=['auto','fixed'], default='auto')
    parser.add_argument('--tilt',type=float,default=0.,help='TCP inclination from the surface normal, degrees')
    parser.add_argument('--tilt-direction',type=float,default=0.,help='Inclination direction in the nominal tool XY plane, degrees')
    parser.add_argument('--max-tilt',type=float,default=45.,help='Maximum inclination during automatic search, degrees')
    parser.add_argument('--orientation-timeout',type=float,default=120.,help='Approximate automatic planning budget, seconds')
    parser.add_argument("--speed", type=float, default=0.02, help="Maximum pass speed, m/s")
    parser.add_argument("--scaling", type=float, default=0.05, help="Joint velocity/acceleration factor")
    parser.add_argument("--mode", choices=["preview", "plan", "execute"], default="preview")
    parser.add_argument("--once", action="store_true", help="Exit after publishing preview")
    parser.add_argument("--output", default="diagnostics/welding", help="Directory for JSON reports")
    parser.add_argument('--scan-report')
    parser.add_argument('--use-scene-workpiece',action='store_true',help='Use the current MoveIt/RViz workpiece directly, as reviewed by the operator')
    parser.add_argument('--scan-calibration')
    parser.add_argument('--scanner-url', default='http://127.0.0.1:8158')
    args = parser.parse_args()
    if args.scan_report and not args.scan_calibration:
        parser.error('--scan-report requires --scan-calibration')
    if args.use_scene_workpiece and args.scan_report:
        parser.error('Choose current scene geometry or scan receipt, not both')
    if (not all(math.isfinite(v) for v in (args.tilt,args.tilt_direction,args.max_tilt,args.orientation_timeout))
            or not 0<=args.tilt<=args.max_tilt<=60 or not 5<=args.orientation_timeout<=300):
        parser.error('Require 0<=tilt<=max-tilt<=60 degrees and orientation-timeout in 5..300 seconds')
    finite = [args.begin, args.end, args.offset, args.clearance, args.step, args.roll, args.speed, args.scaling]
    if (not all(math.isfinite(v) for v in finite) or not 0 <= args.begin < args.end <= 1
            or not 0 < args.offset <= 0.5 or not 0 < args.clearance <= 0.5
            or not 0.001 <= args.step <= 0.02 or not 0 < args.speed <= 0.1
            or not 0 < args.scaling <= 1.0):
        parser.error("Require 0<=begin<end<=1; offset/clearance (0,0.5]; step [0.001,0.02]; speed (0,0.1]; scaling (0,1]")
    return args


def main(runner_type=None):
    args = arguments()
    # Keep the ROS context alive long enough to cancel an action on Ctrl+C.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = (runner_type or WeldingRunner)(args)
    node.report['planning_source']='moveit_scene' if args.use_scene_workpiece else 'scan_receipt' if args.scan_report else 'default_scene'
    code = 0
    try:
        edges = node.read_object()
        geometry = node.geometry
        node.report['shape'] = geometry.shape
        if args.list:
            for face, contour in geometry.contours.items():
                center = np.mean(contour.vertices[:-1], axis=0)
                print(f"face {face:8s} perimeter {contour.length:.3f} m  center {center.round(4).tolist()}")
            print("Edges:")
            for edge in edges:
                print(f"{edge.id:24s} {edge.length:.3f} m  {edge.start.round(4).tolist()} -> {edge.end.round(4).tolist()}")
            return 0
        if args.edge:
            selected = geometry.select(args.edge)
            seams = selected
        else:
            face = args.face or (geometry.selectable_seams[0] if geometry.selectable_seams else
                                'z_plus' if 'z_plus' in geometry.contours else next(iter(geometry.contours)))
            parent=geometry.contour(face).face_id or face
            selected = [e for e in edges if parent in e.faces]
            seams = [geometry.contour(face)]
        paths = [(edge, *edge.path(args.begin, args.end, args.offset, args.step, math.radians(args.roll)))
                 for edge in seams]
        if args.mode=='preview':
            paths=[(edge,seam,points,tilted_orientation(q,args.tilt,args.tilt_direction)) for edge,seam,points,q in paths]
        node.report["selection"] = [{"edge": edge.id, "begin": args.begin, "end": args.end,
                                     "seam": seam.tolist(), "tool_path": points.tolist(),
                                     "orientation_xyzw": q.tolist()} for edge, seam, points, q in paths]
        node.report["settings"] = {k: getattr(args, k) for k in ("offset", "clearance", "step", "speed", "scaling", "roll",
            'orientation_mode','tilt','tilt_direction','max_tilt','orientation_timeout')}
        node.publish_markers(edges, selected, paths)
        if args.mode == "preview":
            node.report["status"] = "preview"
            node.get_logger().info("Preview published on /welding/markers. No motion requested. Ctrl+C to exit.")
            if not args.once:
                rclpy.spin(node)
        else:
            stages = node.plan(paths)
            if args.mode == "execute":
                node.execute(stages)
            node.get_logger().info(f"Welding operation {node.report['status']}")
        # Allow transient publishers to match existing RViz subscribers.
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.report["status"] = "interrupted"
        code = 0 if args.mode == "preview" else 130
    except Exception as error:
        node.report.update(status="failed", error=str(error))
        node.get_logger().error(str(error))
        code = 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node.active_goal is not None:
            cancel = node.active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=5)
        if not args.list:
            directory = Path(args.output)
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
            path.write_text(json.dumps(node.report, indent=2) + "\n")
            print(f"Report: {path}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
