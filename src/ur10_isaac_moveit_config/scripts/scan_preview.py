#!/usr/bin/env python3
"""Scanner display and optional automatic, unverified CAD placement."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from scan_job import fetch_bundle
from rangevision import Bridge
from auto_workpiece import AutoWorkpiece
from scan_stream import CaptureLoop, scanner_request
from scan_view import level_rotation, table_height_anchor, yaw_about_point, scaled_preview_points
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
import math
import subprocess


def cloud_message(points, frame):
    points = np.asarray(points,dtype='<f4')
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():
        raise ValueError('Invalid preview cache or cloud')
    return PointCloud2(header=Header(frame_id=frame),height=1,width=len(points),
        fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,n in enumerate(('x','y','z'))],is_bigendian=False,
        point_step=12,row_step=12*len(points),data=points.tobytes(),is_dense=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scanner-url', default='windows-local')
    parser.add_argument('--continuous', action='store_true', help='Acquire successive single-view scans')
    parser.add_argument('--view-only', action='store_true', help='Only display scans; do not start acquisition')
    parser.add_argument('--auto-fit',action='store_true',help='Align unverified startup CAD while idle; never authorize welding')
    parser.add_argument('--relevel', action='store_true', help='Re-estimate and save the visual reference plane')
    parser.add_argument('--rebase-table', action='store_true', help='Once, place the robust lower cloud boundary on the table; save visual height reference')
    parser.add_argument('--stop-with-pid', type=int, help='Stop when this existing launch process exits')
    parser.add_argument('--center', nargs=3, type=float, default=None, metavar=('X', 'Y', 'Z'),
                        help='Uncalibrated preview center in world, metres')
    args = parser.parse_args()
    table = json.loads((Path(get_package_share_directory('jaka_zu12_moveit_config'))/'config/table_grid.json').read_text())
    metric_scale=float(table.get('preview_metric_scale',{}).get('factor',1.))
    scaled_preview_points(np.zeros((1,3)),np.zeros(3),metric_scale)
    region = table['scanner_region']
    def address_xy(address):
        letters = ''.join(c for c in address if c.isalpha())
        column = 0
        for letter in letters: column = column*26+ord(letter)-ord('A')+1
        row = int(address[len(letters):])
        g = table['grid']
        return np.array([g['offset_x_mm']+(column-1)*g['pitch_mm'],
                         g['offset_y_mm']+(row-1)*g['pitch_mm'],0.])*.001
    region_center = (address_xy(region['from'])+address_xy(region['to']))/2
    yaw = math.radians(table['world_from_table']['yaw_deg'])
    rotation = np.array([[math.cos(yaw),-math.sin(yaw),0.],
                         [math.sin(yaw),math.cos(yaw),0.],[0.,0.,1.]])
    if args.center is None:
        args.center = (rotation@region_center+np.array(table['world_from_table']['translation_m'])
                       +np.array(table.get('preview_translation_world_m',[0.,0.,0.]))).tolist()
    if not np.isfinite(args.center).all():
        parser.error('Preview center must be finite')
    rclpy.init(args=[])
    node = Bridge('scanner_unregistered_preview')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(PointCloud2, '/scanner/preview_points', qos)
    status_publisher = node.create_publisher(String, '/scanner/preview_status', qos)
    loop = CaptureLoop(lambda *request: scanner_request(args.scanner_url, *request),
                       continuous=args.continuous and not args.view_only)
    root = next(p for p in Path(__file__).resolve().parents if (p / 'run_scan_preview.sh').is_file())
    if args.scanner_url=='windows-local' and not args.view_only:
        try:
            script=subprocess.check_output(['wslpath','-w',str(root/'integrations/ensure_scanner.ps1')],text=True).strip()
            ready=subprocess.run(['powershell.exe','-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass','-File',script],
                                 capture_output=True,timeout=50)
            if ready.returncode:node.get_logger().warning('Windows scanner startup: '+ready.stderr.decode('utf-8',errors='replace')[-1500:])
            else:node.get_logger().info('Windows scanner ready: '+ready.stdout.decode('utf-8',errors='replace'))
        except (OSError,subprocess.TimeoutExpired) as error:
            node.get_logger().warning('Windows scanner startup: '+str(error))
    auto_workpiece=AutoWorkpiece(node,root) if args.auto_fit and not args.view_only else None
    pending_fit=None
    next_fit=0.
    anchor_path = root / 'scans/preview-anchor.json'
    cache_path = root / 'scans/preview-cloud-cache.npz'
    restart_path = root / 'scans/workpiece-scan-request'
    restart_ack = root / 'scans/workpiece-scan-applied'
    applied_restart = restart_ack.read_text().strip() if restart_ack.is_file() else None
    orientation_path = root / 'scans/preview-orientation.json'
    orientation = None
    if orientation_path.is_file() and not args.relevel:
        orientation = json.loads(orientation_path.read_text())
    anchor = None
    anchor_frame = None
    height_reference = None
    height_rebased = False
    if anchor_path.is_file():
        saved = json.loads(anchor_path.read_text())
        candidate = np.asarray(saved['camera_origin_m'], dtype=float)
        if candidate.shape == (3,) and np.isfinite(candidate).all():
            anchor = candidate
            anchor_frame = saved['frame_signature']
            height_reference = saved.get('height_reference')
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'world'
    transform.child_frame_id = 'scanner_preview_UNCALIBRATED'
    initial_rotation = np.eye(3)
    if orientation and orientation.get('frame_signature')==anchor_frame and orientation.get('table_yaw_deg')==table['world_from_table']['yaw_deg']:
        initial_rotation = np.array(orientation['camera_to_world_rotation'])
    yaw_adjustment = table.get('preview_yaw_adjustment',{})
    def set_pose(camera_rotation):
        effective_rotation,effective_center = yaw_about_point(camera_rotation,args.center,
            yaw_adjustment.get('angle_deg',0.),yaw_adjustment.get('pivot_world_m',[0.,0.,0.]))
        q = Rotation.from_matrix(effective_rotation).as_quat()
        for axis,value in zip('xyzw',q):setattr(transform.transform.rotation,axis,float(value))
        for axis,value in zip('xyz',effective_center):setattr(transform.transform.translation,axis,float(value))
    set_pose(initial_rotation)
    transform.header.stamp = node.get_clock().now().to_msg()
    broadcaster.sendTransform(transform)
    node.get_logger().warning(f"UNCALIBRATED PREVIEW: first-cloud anchor placed at scanner region {region['from']}..{region['to']} center {args.center}; camera pose is unmeasured. Automatic CAD fitting is preliminary and never authorizes robot motion.")
    message = None
    next_fetch = 0.
    previous = None
    next_publish=0.
    updated_at = None
    displayed_capture = None
    if (cache_path.is_file() and anchor is not None and orientation is not None
            and orientation.get('frame_signature')==anchor_frame
            and orientation.get('table_yaw_deg')==table['world_from_table']['yaw_deg']):
        try:
            with np.load(cache_path) as cache:
                stamp = float(cache['updated_unix'])
                if str(cache['frame_signature'])==anchor_frame and 0<stamp<=time.time()+30:
                    updated_at = stamp
                    displayed_capture = str(cache['capture_id'])
                    # Restore timestamps only. Publishing a cached cloud before
                    # the scanner confirms it can visibly restore an old pose.
                    if args.view_only and time.time()-stamp<600:
                        message = cloud_message(scaled_preview_points(cache['camera_points_m'],anchor,metric_scale),transform.child_frame_id)
                        message.header.stamp = node.get_clock().now().to_msg()
                        publisher.publish(message)
        except (OSError,ValueError,KeyError) as error:
            node.get_logger().warning('Preview cache ignored: '+str(error))
    last_status = None
    last_error = None
    try:
        while rclpy.ok():
            if args.stop_with_pid and not Path(f'/proc/{args.stop_with_pid}').exists():
                break
            if time.monotonic() >= next_fetch:
                next_fetch = time.monotonic() + 5.
                try:
                    restart_name = None
                    if loop.continuous and restart_path.is_file():
                        requested = restart_path.read_text().strip()
                        if requested and requested != applied_restart:
                            restart_name = requested
                            pending_fit = None
                    prior_failure = loop.last_failure
                    bundle = loop.poll(lambda: fetch_bundle(args.scanner_url), restart_name)
                    if loop.last_failure and loop.last_failure != prior_failure:
                        node.get_logger().warning(f'Scanner rejected capture: {loop.last_failure}')
                    if restart_name and loop.applied_restart == restart_name:
                        temporary_ack = restart_ack.with_suffix('.tmp')
                        temporary_ack.write_text(restart_name)
                        temporary_ack.replace(restart_ack)
                        applied_restart = restart_name
                        node.get_logger().info(f'Scan request handled for {restart_name}')
                    last_error = None
                    if bundle is not None and bundle['capture_id'] != previous:
                        points = np.asarray(bundle['camera_points_mm'], dtype=float) * .001
                        if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
                            raise ValueError('Scanner returned an invalid point cloud')
                        if (orientation is None or orientation.get('frame_signature') != bundle['frame_signature']
                                or orientation.get('table_yaw_deg') != table['world_from_table']['yaw_deg']):
                            orientation = level_rotation(points, table['world_from_table']['yaw_deg'],
                                table['preview_orientation']['nominal_down_deg'])
                            orientation.update(frame_signature=bundle['frame_signature'],
                                               reference_capture_id=bundle['capture_id'])
                            orientation_path.parent.mkdir(parents=True, exist_ok=True)
                            temporary = orientation_path.with_suffix('.tmp')
                            temporary.write_text(json.dumps(orientation, indent=2))
                            temporary.replace(orientation_path)
                            node.get_logger().info('Saved visual leveling: ' + json.dumps(orientation))
                        camera_rotation = np.asarray(orientation['camera_to_world_rotation'], dtype=float)
                        if (camera_rotation.shape != (3,3) or not np.isfinite(camera_rotation).all()
                                or not np.allclose(camera_rotation.T@camera_rotation, np.eye(3), atol=1e-6)
                                or not np.isclose(np.linalg.det(camera_rotation), 1.)):
                            raise ValueError('Invalid saved visual rotation; rerun with --relevel')
                        set_pose(camera_rotation)
                        transform.header.stamp = node.get_clock().now().to_msg()
                        broadcaster.sendTransform(transform)
                        # Freeze the origin across captures and restarts, so translations
                        # remain visible. This is still not robot registration.
                        if anchor is None or anchor_frame != bundle['frame_signature']:
                            anchor = np.median(points, axis=0)
                            anchor_frame = bundle['frame_signature']
                            anchor_path.parent.mkdir(parents=True, exist_ok=True)
                            temporary = anchor_path.with_suffix('.tmp')
                            temporary.write_text(json.dumps(dict(frame_signature=anchor_frame,
                                camera_origin_m=anchor.tolist(), registered=False), indent=2))
                            temporary.replace(anchor_path)
                        if args.rebase_table and not height_rebased:
                            backup = anchor_path.with_name('preview-anchor.before-height-'+str(time.time_ns())+'.json')
                            backup.write_bytes(anchor_path.read_bytes())
                            scaled_anchor,height_reference = table_height_anchor(points*metric_scale,anchor*metric_scale,camera_rotation,
                                float(table['world_from_table']['translation_m'][2])-args.center[2])
                            anchor=scaled_anchor/metric_scale
                            height_reference.update(capture_id=bundle['capture_id'],created_unix=time.time(),
                                                    backup_file=str(backup))
                            temporary = anchor_path.with_suffix('.tmp')
                            temporary.write_text(json.dumps(dict(frame_signature=anchor_frame,
                                camera_origin_m=anchor.tolist(),registered=False,height_reference=height_reference),indent=2))
                            temporary.replace(anchor_path)
                            height_rebased = True
                            node.get_logger().info('Saved visual table height: '+json.dumps(height_reference))
                        local_points=scaled_preview_points(points,anchor,metric_scale)
                        message = cloud_message(local_points,transform.child_frame_id)
                        # One completed physical capture is immutable, even if
                        # scanner UI metadata changes its export identity.
                        previous = bundle['capture_id']
                        if displayed_capture!=bundle['capture_id'] or updated_at is None:updated_at=time.time()
                        displayed_capture = bundle['capture_id']
                        stamp_path=root/'scans/scan_status.json'
                        temporary_stamp=stamp_path.with_suffix('.tmp')
                        temporary_stamp.write_text(json.dumps(dict(updated_unix=updated_at,capture_id=displayed_capture)))
                        temporary_stamp.replace(stamp_path)
                        if auto_workpiece:
                            q=transform.transform.rotation
                            effective_rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
                            center=np.array([getattr(transform.transform.translation,a) for a in 'xyz'])
                            pending_fit=(bundle,local_points@effective_rotation.T+center)
                        temporary_cache = cache_path.with_suffix('.tmp.npz')
                        np.savez(temporary_cache,camera_points_m=points,frame_signature=bundle['frame_signature'],
                                 capture_id=displayed_capture,updated_unix=updated_at)
                        temporary_cache.replace(cache_path)
                        node.get_logger().info(f"Preview: {len(points)} points, capture {bundle['capture_id']}")
                        print(json.dumps({'capture_id': bundle['capture_id'], 'point_count': len(points),
                                          'registered': False}), flush=True)
                except Exception as error:
                    # Do not keep displaying an obsolete capture after an export fails.
                    # Keep the last accepted scan on transient/stale exports.
                    # Neither re-read the disk cache nor replace it on failure.
                    node.get_logger().warning(f'Scanner preview unavailable: {error}')
                    last_error = str(error)
                status = dict(state=loop.state, continuous=loop.continuous,
                              paused=loop.paused, error=last_error or loop.last_failure, updated_unix=updated_at,
                              capture_id=loop.owned_capture, registered=False)
                status['displayed_capture_id'] = displayed_capture
                status.update(scanner_region=region, preview_center_world_m=[getattr(transform.transform.translation,a) for a in 'xyz'],
                              placement_method='region_center_with_saved_manual_translation', orientation_calibrated=False,
                              preview_orientation=orientation)
                status['manual_translation_world_m'] = table.get('preview_translation_world_m',[0.,0.,0.])
                status['preview_yaw_adjustment'] = yaw_adjustment
                status['height_reference'] = height_reference
                status['metric_scale_correction'] = table.get('preview_metric_scale',{'factor':1.})
                status_publisher.publish(String(data=json.dumps(status)))
                notice = (loop.state, loop.paused, last_error)
                if notice != last_status:
                    node.get_logger().info('Acquisition status: ' + json.dumps(status))
                    last_status = notice
            if message is not None and time.monotonic()>=next_publish:
                next_publish=time.monotonic()+.5
                message.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(message)
            if pending_fit is not None and time.monotonic()>=next_fit:
                next_fit=time.monotonic()+10.
                try:
                    if auto_workpiece.update(*pending_fit):pending_fit=None
                except Exception as error:
                    auto_workpiece.status('update_failed',reason=str(error))
            rclpy.spin_once(node, timeout_sec=.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
