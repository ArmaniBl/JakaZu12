#!/usr/bin/env python3
"""File bridge for a fixed RangeVision scanner. Never sends robot motion goals."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.action import ActionClient
from rosidl_runtime_py.convert import message_to_ordereddict
from action_msgs.msg import GoalStatusArray, GoalStatus
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import Header
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.action import ExecuteTrajectory

def signature(obj):
    # CDR padding bytes are not a stable content signature across service calls.
    content = json.dumps(message_to_ordereddict(obj), sort_keys=True, separators=(',', ':'))
    return hashlib.sha256(content.encode()).hexdigest()


class Bridge(Node):
    def __init__(self,name='rangevision_bridge'):
        super().__init__(name)
        self.status_seen = False
        self.busy = False
        self.action_states = {}
        self.samples = []
        self.execution = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.execution_status = self.create_subscription(GoalStatusArray, '/execute_trajectory/_action/status',
                                 lambda msg: self.status('moveit', msg),
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(GoalStatusArray,
                                 '/jaka_zu12_controller/follow_joint_trajectory/_action/status',
                                 lambda msg: self.status('driver', msg),
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/joint_states', self.joints, 10)

    def status(self, source, msg):
        if source == 'moveit':
            self.status_seen = True
        self.action_states[source] = any(
            s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                         GoalStatus.STATUS_CANCELING) for s in msg.status_list)
        self.busy = any(self.action_states.values())

    def joints(self, msg):
        wanted = [f'joint_{i}' for i in range(1, 7)]
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions for j in wanted):
            now=time.monotonic()
            self.samples.append((now, np.array([positions[j] for j in wanted])))
            # The driver publishes at 125 Hz. A 100-message cap retained only
            # 0.8 seconds, making the required one-second observation impossible.
            self.samples = [sample for sample in self.samples if now-sample[0] <= 3.]

    def require_idle(self):
        # Observe real feedback and MoveIt's execution status before changing geometry.
        if not self.execution.wait_for_server(timeout_sec=5):
            raise ValueError('MoveIt execution server is unavailable. Keep run_real.sh running and check its log')
        self.samples = []
        matched_at = None
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if self.busy:
                raise ValueError('MoveIt is executing; stop the job before scan alignment')
            if self.count_publishers('/execute_trajectory/_action/status'):
                if matched_at is None:matched_at=time.monotonic()
                if time.monotonic()-matched_at>=2:break
            else:
                matched_at=None
        publishers=self.get_publishers_info_by_topic('/execute_trajectory/_action/status')
        if matched_at is None or time.monotonic()-matched_at<2 or not publishers:
            raise ValueError('MoveIt status subscription is not matched; check ROS_DOMAIN_ID and DDS settings')
        if not self.status_seen:
            # A fresh ROS action server publishes status on goal transitions,
            # not necessarily an empty heartbeat before its first goal. Require
            # matched reliable transient-local history before accepting silence.
            if not all(p.qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL
                       and p.qos_profile.reliability==ReliabilityPolicy.RELIABLE for p in publishers):
                raise ValueError('MoveIt status history is unavailable; cannot confirm idle state')
        if len(self.samples) < 5 or self.samples[-1][0]-self.samples[0][0] < 1:
            raise ValueError('No continuous JAKA joint feedback')
        if time.monotonic()-self.samples[-1][0] > .3:
            raise ValueError('Stale robot feedback')
        positions = np.array([p for _, p in self.samples])
        if not np.isfinite(positions).all() or np.max(np.ptp(positions, axis=0)) > .001:
            raise ValueError('Robot is moving; scene update rejected')

    def call(self, srv, name, request):
        client = self.create_client(srv, name)
        try:
            if not client.wait_for_service(timeout_sec=5):
                raise ValueError(f'Service unavailable: {name}')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15)
            if not future.done():
                raise ValueError(f'Service timed out: {name}')
            return future.result()
        finally:
            self.destroy_client(client)

    def object(self, object_id):
        if object_id == 'floor':
            raise ValueError('Cannot align the supporting floor as a workpiece')
        scene = self.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request()).scene
        obj = next((o for o in scene.world.collision_objects if o.id == object_id), None)
        if obj is None or obj.header.frame_id != 'world':
            raise ValueError('Workpiece must exist in the world frame')
        return obj

    def publish(self, points):
        publisher = self.create_publisher(PointCloud2, '/rangevision/points',
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        points = np.asarray(points, dtype='<f4')
        msg = PointCloud2(header=Header(frame_id='world'), height=1, width=len(points),
                         fields=[PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                 for i, n in enumerate(('x', 'y', 'z'))],
                         is_bigendian=False, point_step=12, row_step=12*len(points),
                         data=points.tobytes(), is_dense=True)
        print('Publishing /rangevision/points in world; Ctrl+C to stop', flush=True)
        while rclpy.ok():
            msg.header.stamp = self.get_clock().now().to_msg()
            publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=1)


def calibrate(args):
    from welding_sim.scan_alignment import digest, rigid_fit, transform_points

    data = json.loads(Path(args.landmarks).read_text())
    # Independent check landmarks must not be used for fitting.
    transform = rigid_fit(np.array(data['scanner_mm'])*.001, data['world_m'])
    source, target = np.array(data['check_scanner_mm'])*.001, np.array(data['check_world_m'])
    if source.shape != target.shape or source.ndim != 2 or source.shape[0] < 4 or source.shape[1] != 3:
        raise ValueError('At least four independent validation landmarks required')
    # Reject accidentally reusing the fitting measurements as validation.
    fitting = np.array(data['scanner_mm'])*.001
    if any(np.min(np.linalg.norm(fitting-point, axis=1)) < .001 for point in source):
        raise ValueError('Validation landmarks must differ from fitting landmarks')
    errors = np.linalg.norm(transform_points(source, transform)-target, axis=1)
    if not np.isfinite(errors).all() or max(errors) > .001:
        raise ValueError('Independent calibration error exceeds 1 mm')
    result = {'scanner_to_world': transform.tolist(), 'frame': 'world',
              'validation_max_error_m': float(max(errors)), 'created_unix': time.time(),
              'landmarks_sha256': digest(args.landmarks), 'fixed_scanner': True}
    if getattr(args, 'frame_signature', None):
        result['frame_signature'] = args.frame_signature
    Path(args.output).write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


def world_cloud(path, units, calibration):
    from welding_sim.scan_alignment import digest, read_cloud, transform_points, validate_transform

    before = (digest(path), digest(calibration))
    config = json.loads(Path(calibration).read_text())
    if (config.get('frame') != 'world' or config.get('fixed_scanner') is not True
            or not 0 <= config.get('validation_max_error_m', float('inf')) <= .001):
        raise ValueError('Calibrate and validate the fixed scanner against robot world first')
    matrix = validate_transform(config['scanner_to_world'])
    result = transform_points(read_cloud(path, units), matrix)
    if before != (digest(path), digest(calibration)):
        raise ValueError('Scan/calibration changed while reading; use a finished export')
    return result


def compare(node, cloud, calibration, units, roi, object_id):
    from welding_sim.scan_alignment import align_translation, digest
    from welding_sim.scan_model import sample_object

    node.require_idle()
    obj = node.object(object_id)
    points = world_cloud(cloud, units, calibration)
    low, high = np.array(roi)[::2], np.array(roi)[1::2]
    if not np.isfinite(roi).all() or np.any(low >= high):
        raise ValueError('ROI must be xmin xmax ymin ymax zmin zmax in world metres')
    points = points[np.all((points >= low) & (points <= high), axis=1)]
    # Deterministic voxel reduction for large exported scans.
    _, unique = np.unique(np.floor(points/.001), axis=0, return_index=True)
    points = points[unique]
    if len(points) > 100000:
        raise ValueError('ROI is too large; isolate only the measured workpiece')
    model, normals = sample_object(obj)
    result = align_translation(points, model, normals)
    result.update(object=object_id, object_sha256=signature(obj), cloud=str(Path(cloud).resolve()),
                  cloud_sha256=digest(cloud), calibration=str(Path(calibration).resolve()),
                  calibration_sha256=digest(calibration), units=units, roi=roi,
                  created_unix=time.time(), frame='world', mode='translation_only',
                  needs_update=bool(np.linalg.norm(result['translation_m']) > .001))
    return result, obj


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    cal = commands.add_parser('calibrate')
    cal.add_argument('--landmarks', required=True);cal.add_argument('--output', required=True)
    cal.add_argument('--frame-signature', help='Measured stable frame signature from the scanner bridge')
    for name in ('publish', 'compare'):
        cmd = commands.add_parser(name)
        cmd.add_argument('--cloud', required=True)
        cmd.add_argument('--units', choices=('mm', 'm'), required=True)
        cmd.add_argument('--calibration', required=True)
        if name == 'compare':
            cmd.add_argument('--roi', nargs=6, type=float, required=True)
            cmd.add_argument('--object', default='welding_workpiece')
            cmd.add_argument('--report', required=True)
    apply = commands.add_parser('apply')
    apply.add_argument('--report', required=True)
    args = parser.parse_args()
    node = None
    try:
        if args.command == 'calibrate':
            calibrate(args);return
        rclpy.init(args=[]);node = Bridge()
        if args.command == 'publish':
            node.publish(world_cloud(args.cloud, args.units, args.calibration))
        elif args.command == 'compare':
            result, _ = compare(node, args.cloud, args.calibration, args.units, args.roi, args.object)
            Path(args.report).write_text(json.dumps(result, indent=2))
            print(json.dumps(result, indent=2))
        else:
            approved = json.loads(Path(args.report).read_text())
            if not 0 <= time.time()-approved['created_unix'] <= 300:
                raise ValueError('Comparison expired (5 minutes); compare again')
            result, obj = compare(node, approved['cloud'], approved['calibration'], approved['units'],
                                  approved['roi'], approved['object'])
            for key in ('object_sha256', 'cloud_sha256', 'calibration_sha256'):
                if result[key] != approved[key]:
                    raise ValueError('Scene/scan/calibration changed since comparison; compare again')
            if not np.allclose(result['translation_m'], approved['translation_m'], atol=1e-6):
                raise ValueError('Alignment changed since approval')
            node.require_idle()
            if signature(node.object(obj.id)) != result['object_sha256']:
                raise ValueError('Workpiece changed; compare again')
            moved = deepcopy(obj)
            # Translation is in world, independent of the object's local rotation.
            moved.pose.position.x += result['translation_m'][0]
            moved.pose.position.y += result['translation_m'][1]
            moved.pose.position.z += result['translation_m'][2]
            request = ApplyPlanningScene.Request()
            request.scene.is_diff = True;request.scene.robot_state.is_diff = True
            request.scene.world.collision_objects = [moved]
            response = node.call(ApplyPlanningScene, '/apply_planning_scene', request)
            if not response.success:
                raise ValueError('MoveIt rejected the updated workpiece')
            print('Workpiece translated. Discard old plans and create a new welding plan. No motion sent.')
    except (ValueError, KeyError, OSError, TypeError, IndexError) as error:
        parser.exit(1, f'RangeVision: {error}\n')
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
