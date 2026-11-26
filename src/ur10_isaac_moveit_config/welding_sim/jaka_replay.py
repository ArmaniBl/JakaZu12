"""Plan welding with the JAKA chain and replay joint trajectories in Isaac."""
from copy import deepcopy
import math
import time
from types import SimpleNamespace
from xml.etree.ElementTree import fromstring

import numpy as np

from moveit_msgs.msg import CollisionObject, DisplayTrajectory, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from sensor_msgs.msg import JointState

from .joint_mapping import convert_trajectory, replay_branch, ur10_to_jaka
from .profiles import JAKA, UR_JOINTS
from .runner import TABLE_CLEARANCE_M, TableClearance, WeldingRunner, object_geometry, pose_transform
from .table_limits import JAKA_TABLE_TOP_M

PLANNER_NAMESPACE = '/jaka_sim_planner'
JAKA_READY = (-math.pi / 2, 1.6, -1.3, 1.25, math.pi / 2, 0.)


class JakaReplayRunner(WeldingRunner):
    """Keep the isolated JAKA planner separate from the UR10 controller."""

    welding_profile = JAKA

    def __init__(self, args):
        super().__init__(args)
        self.planning_jaka = False
        self.jaka_positions = None
        self.jaka_states = self.create_publisher(JointState, f'{PLANNER_NAMESPACE}/joint_states', 10)
        self.jaka_timer = self.create_timer(.05, self.publish_jaka_feedback)

    def call(self, service, name, request, timeout=30.):
        if self.planning_jaka:
            name = PLANNER_NAMESPACE + name
        return super().call(service, name, request, timeout)

    def receive_state(self, message):
        if not self.planning_jaka:
            super().receive_state(message)

    def publish_jaka_feedback(self):
        if self.planning_jaka and self.jaka_positions is not None:
            message = JointState(name=list(JAKA.joints), position=self.jaka_positions)
            message.header.stamp = self.get_clock().now().to_msg()
            self.jaka_states.publish(message)
            super().receive_state(message)

    def jaka_scene(self, ur_scene):
        deadline = time.monotonic() + 8.
        while True:
            scene = self.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request()).scene
            if any(obj.id == 'floor' for obj in scene.world.collision_objects):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('JAKA planning table is unavailable; restart run_sim')
            time.sleep(.15)
        source_floor = self.planning_floor
        surface = pose_transform(source_floor.pose) @ pose_transform(source_floor.primitive_poses[0])
        dz = JAKA_TABLE_TOP_M - (surface[2, 3] + source_floor.primitives[0].dimensions[2] / 2)
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = request.scene.robot_state.is_diff = True

        def planning_object(obj):
            return obj.id == 'floor' or obj.id.startswith('workspace_')

        root_objects = {obj.id: obj for obj in ur_scene.world.collision_objects if not planning_object(obj)}
        changes = [CollisionObject(id=obj.id, header=obj.header, operation=CollisionObject.REMOVE)
                   for obj in scene.world.collision_objects
                   if not planning_object(obj) and obj.id not in root_objects]
        for obj in root_objects.values():
            adjusted = deepcopy(obj)
            adjusted.pose.position.z += float(dz)
            changes.append(adjusted)
        request.scene.world.collision_objects = changes
        if not self.call(ApplyPlanningScene, '/apply_planning_scene', request).success:
            raise RuntimeError('The isolated JAKA planner rejected the workcell')
        return self.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request()).scene, dz

    def plan(self, paths):
        if set(self.profile.joints) != set(UR_JOINTS):
            raise RuntimeError('JAKA replay requires the six standard UR10 joint names')
        self.ur_profile = self.profile
        self.ur_scene, self.ur_floor, self.ur_return = self.initial_scene, self.planning_floor, self.return_pose
        ur_start = self.settled_state()
        angles = dict(zip(ur_start.joint_state.name, ur_start.joint_state.position))
        self.ur_positions = [angles[name] for name in UR_JOINTS]
        geometry, display = self.geometry, self.display
        self.jaka_preview = None
        self.replay_preview = None
        replay_block = None
        self.planning_jaka = True
        self.profile = JAKA
        self.display = SimpleNamespace(publish=lambda _: None)
        self.get_logger().info('Planning weld with the JAKA ZU12 model')
        try:
            self.initial_scene, dz = self.jaka_scene(self.ur_scene)
            self.planning_floor = next(obj for obj in self.initial_scene.world.collision_objects if obj.id == 'floor')
            for candidate in (ur10_to_jaka(self.ur_positions), list(JAKA_READY)):
                state = RobotState(joint_state=JointState(name=list(JAKA.joints), position=candidate), is_diff=True)
                request = GetStateValidity.Request(robot_state=state)
                if self.call(GetStateValidity, '/check_state_validity', request).valid:
                    self.jaka_positions = candidate
                    break
            else:
                raise RuntimeError('No valid JAKA start posture for the current scene')

            obj = next(obj for obj in self.initial_scene.world.collision_objects if obj.id == self.args.object)
            self.geometry = object_geometry(obj, self.ur_profile.frame)
            shift = np.array([0., 0., dz])
            adjusted = []
            for edge, seam, points, q in paths:
                mapped_edge = (self.geometry.contour(edge.id) if edge.id in self.geometry.contours
                               else self.geometry.select([edge.id])[0])
                adjusted.append((mapped_edge, seam + shift, points + shift, q))
            self.report.update(robot_profile=JAKA.payload(), replay_profile=self.ur_profile.payload(),
                               vertical_clearance_m=JAKA.vertical_clearance_m, planning_source='jaka_sim_planner')
            self.publish_jaka_feedback()
            super().plan(adjusted)
        except RuntimeError as error:
            if self.jaka_preview is None:
                raise
            if self.args.mode != 'plan':
                raise RuntimeError(f'JAKA route planned, but Isaac replay is blocked: {error}') from error
            replay_block = str(error)
        finally:
            self.planning_jaka = False
            self.profile = self.ur_profile
            self.initial_scene, self.planning_floor, self.return_pose = self.ur_scene, self.ur_floor, self.ur_return
            self.geometry, self.display = geometry, display

        if replay_block is not None:
            failures = self.report.get('validation_failures', [])
            self.report = self.jaka_preview
            self.report.update(status='planned', validation_failures=failures,
                               replay={'status': 'blocked', 'error': replay_block})
            self.get_logger().warning(f'JAKA plan verified. Isaac execution is blocked: {replay_block}')
            if self.replay_preview is not None:
                self.display.publish(self.replay_preview)
            return []

        state, stages, targets, clearance = self.replay_candidate
        self.body_clearance = clearance
        self.cartesian_targets = {}
        self.joint_targets = targets
        self.display.publish(DisplayTrajectory(model_id=self.ur_profile.model, trajectory_start=state,
                                               trajectory=[trajectory for _, trajectory in stages]))
        return stages

    def validate_candidate_replay(self, stages):
        """Try the validated JAKA motion as-is on the UR10 planning model."""
        self.report['jaka_stages'] = self.report['stages']
        self.report['jaka_body_collision_checks'] = self.report.pop('body_collision_checks', [])
        self.report['jaka_plan_status'] = 'planned'
        if self.jaka_preview is None:
            self.jaka_preview = deepcopy(self.report)
        jaka_scene, jaka_floor, jaka_clearance = self.initial_scene, self.planning_floor, self.body_clearance
        self.planning_jaka = False
        self.profile = self.ur_profile
        self.initial_scene, self.planning_floor = self.ur_scene, self.ur_floor
        try:
            current = self.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request()).scene
            original = self.ur_scene
            if (sorted(current.world.collision_objects, key=lambda obj: obj.id)
                    != sorted(original.world.collision_objects, key=lambda obj: obj.id)
                    or current.robot_state.attached_collision_objects != original.robot_state.attached_collision_objects
                    or current.allowed_collision_matrix != original.allowed_collision_matrix):
                raise RuntimeError('The Isaac scene changed while the JAKA route was planned')
            self.check_world_collisions_enabled(current)
            state = self.settled_state()
            description = self.remote_parameter('move_group', 'robot_description')
            joints = {joint.attrib['name']: joint for joint in fromstring(description).findall('joint')}
            limits = [(float(joints[name].find('limit').attrib['lower']),
                       float(joints[name].find('limit').attrib['upper'])) for name in UR_JOINTS]
            positions = []
            for _, trajectory in stages:
                path = trajectory.joint_trajectory
                indices = [path.joint_names.index(name) for name in JAKA.joints]
                positions.extend([point.positions[j] for j in indices] for point in path.points)
            offsets = replay_branch(self.jaka_positions, self.ur_positions, positions, limits)
            mapped = [(name, convert_trajectory(trajectory, UR_JOINTS, offsets, 1.57))
                      for name, trajectory in stages]
            if self.replay_preview is None and self.args.mode == 'plan':
                first = mapped[0][1].joint_trajectory
                self.replay_preview = DisplayTrajectory(
                    model_id=self.ur_profile.model, trajectory=[trajectory for _, trajectory in mapped],
                    trajectory_start=RobotState(joint_state=JointState(
                        name=list(first.joint_names), position=list(first.points[0].positions))))
            self.body_clearance = TableClearance(description, self.ur_profile, self.ur_floor)
            targets = self.validate_replay(state, mapped)
            self.replay_candidate = (state, mapped, targets, self.body_clearance)
        finally:
            self.planning_jaka = True
            self.profile = JAKA
            self.initial_scene, self.planning_floor, self.body_clearance = jaka_scene, jaka_floor, jaka_clearance
            self.publish_jaka_feedback()

    def validate_replay(self, start, stages):
        state = start
        joint_targets = {}
        self.get_logger().info('Checking the mapped JAKA motion in the Isaac scene')
        try:
            path = stages[0][1].joint_trajectory
            if max(self.start_errors(state, stages[0][1]).values()) > .005:
                target = dict(zip(path.joint_names, path.points[0].positions))
                transfer = self.joint_motion(state, target)
                stages.insert(0, ('replay:align_start', transfer))
                joint_targets['replay:align_start'] = target
            for name, trajectory in stages:
                if max(self.start_errors(state, trajectory).values()) > .005:
                    raise RuntimeError(f'Discontinuous mapped joint positions at {name}')
                self.check_body_path(name, trajectory, state.joint_state)
                state = self.end_state(state, trajectory)
            targets = self.ur_return['joint_positions']
            finish = self.joint_motion(state, targets)
            self.check_body_path('workpiece:return', finish, state.joint_state)
            stages.append(('workpiece:return', finish))
            joint_targets['workpiece:return'] = targets
            clearance = self.body_clearance.minimum(stages, start.joint_state)
            if clearance['clearance_m'] < TABLE_CLEARANCE_M:
                raise RuntimeError(f'Isaac replay tabletop clearance {clearance["clearance_m"] * 1000:.1f} mm')
        except RuntimeError as error:
            self.report['replay'] = {'status': 'blocked', 'error': str(error)}
            raise
        self.report['replay'] = {'status': 'validated', 'table_clearance': clearance}
        self.report['stages'] = []
        for name, trajectory in stages:
            points = trajectory.joint_trajectory.points
            duration = points[-1].time_from_start
            self.report['stages'].append(dict(name=name, points=len(points),
                                              duration_sec=duration.sec + duration.nanosec / 1e9))
        self.report['return_pose'] = deepcopy(self.ur_return)
        return joint_targets
