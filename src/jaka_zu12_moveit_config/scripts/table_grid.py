#!/usr/bin/env python3
"""Addressed tabletop reference overlay. Does not alter collision geometry."""
import json
import math
import os
import fcntl
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from welding_sim.table_limits import JAKA_TABLE_TOP_M, world_point


def column_name(index):
    result = ''
    index += 1
    while index:
        index, remainder = divmod(index - 1, 26)
        result = chr(65 + remainder) + result
    return result


def holes(config):
    table, grid = config['table'], config['grid']
    pitch = grid['pitch_mm']
    nx = math.floor((table['width_mm'] - grid['diameter_mm']/2 - grid['offset_x_mm']) / pitch) + 1
    ny = math.floor((table['depth_mm'] - grid['diameter_mm']/2 - grid['offset_y_mm']) / pitch) + 1
    return [dict(address=f'{column_name(x)}{y+1}', x_mm=grid['offset_x_mm']+x*pitch,
                 y_mm=grid['offset_y_mm']+y*pitch) for y in range(ny) for x in range(nx)]


def make_markers(config, show_tabletop=False):
    markers = []
    def marker(namespace, kind, color):
        item = Marker()
        item.header.frame_id = config['frame']
        item.ns, item.id, item.type = namespace, len(markers), kind
        item.action = Marker.ADD
        item.pose.orientation.w = 1.
        item.color.r, item.color.g, item.color.b, item.color.a = color
        markers.append(item)
        return item
    def text(namespace, value, x, y, z, size=.012, color=(.95,.95,.95,1.)):
        item = marker(namespace, Marker.TEXT_VIEW_FACING, color)
        item.text = value
        item.pose.position = Point(x=x, y=y, z=z)
        item.scale.z = size
    width, depth = config['table']['width_mm']/1000, config['table']['depth_mm']/1000
    if show_tabletop:
        tabletop = marker('tabletop', Marker.CUBE, (.45, .48, .52, 1.))
        tabletop.scale.x, tabletop.scale.y, tabletop.scale.z = width, depth, .10
        tabletop.pose.position = Point(x=width / 2, y=depth / 2, z=-tabletop.scale.z / 2)
    grid = config['grid']
    lattice = holes(config)
    lines = marker('grid_lines', Marker.LINE_LIST, (.66,.69,.72,.7))
    lines.scale.x = .0007
    for x in sorted({h['x_mm']/1000 for h in lattice}):
        lines.points.extend([Point(x=x,y=0.,z=.001),Point(x=x,y=depth,z=.001)])
    for y in sorted({h['y_mm']/1000 for h in lattice}):
        lines.points.extend([Point(x=0.,y=y,z=.001),Point(x=width,y=y,z=.001)])
    border = marker('table_outline', Marker.LINE_STRIP, (.8,.82,.84,1.))
    border.scale.x = .003
    border.points = [Point(x=x,y=y,z=.002) for x,y in [(0.,0.),(width,0.),(width,depth),(0.,depth),(0.,0.)]]
    rings = marker('hole_rings', Marker.LINE_LIST, (.12,.14,.16,1.))
    rings.scale.x = .0015
    radius = grid['diameter_mm']/2000
    for hole in lattice:
        x, y = hole['x_mm']/1000, hole['y_mm']/1000
        for step in range(16):
            for angle in (step*math.tau/16, (step+1)*math.tau/16):
                rings.points.append(Point(x=x+radius*math.cos(angle),y=y+radius*math.sin(angle),z=.002))
        text('hole_addresses', hole['address'], x, y, .017)
    for axis, end, color in [('X',(min(width,.25),0.,.008),(1.,.25,.25,1.)),
                              ('Y',(0.,min(depth,.25),.008),(.25,1.,.25,1.)),
                              ('Z',(0.,0.,.18),(.3,.55,1.,1.))]:
        arrow = marker('table_axes', Marker.ARROW, color)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = .004,.012,.02
        arrow.points = [Point(x=0.,y=0.,z=.008),Point(x=end[0],y=end[1],z=end[2])]
        text('table_axes', 'table '+axis, *end, size=.025, color=color)
    text('table_info', 'A1..X16 | 50 mm | offsets 25 mm: UNVERIFIED', width/2, -.055, .025, size=.023)
    if config.get('scanner_region'):
        region = config['scanner_region']
        addresses = {h['address']:h for h in lattice}
        a, b = addresses[region['from']], addresses[region['to']]
        x0,x1 = sorted((a['x_mm']/1000,b['x_mm']/1000))
        y0,y1 = sorted((a['y_mm']/1000,b['y_mm']/1000))
        outline = marker('scanner_region', Marker.LINE_STRIP, (.15,.85,1.,1.))
        outline.scale.x = .004
        outline.points = [Point(x=x,y=y,z=.004) for x,y in [(x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)]]
        text('scanner_region', f"Scanner {region['from']}..{region['to']} | preview only",
             (x0+x1)/2,y1+.03,.035,size=.02,color=(.15,.85,1.,1.))
    return MarkerArray(markers=markers)


class TableGrid(Node):
    def __init__(self):
        super().__init__('table_grid_reference')
        default = str(Path(get_package_share_directory('ur10_isaac_moveit_config'))/'config/table_grid.json')
        config_path = self.declare_parameter('config', default).value
        config = json.loads(Path(config_path).read_text())
        top_z = self.declare_parameter('top_z', JAKA_TABLE_TOP_M).value
        show_tabletop = self.declare_parameter('show_tabletop', False).value
        self.markers = make_markers(config, show_tabletop)
        for marker in self.markers.markers:
            marker.pose.position.z += top_z - config['world_from_table']['translation_m'][2]
        self.publisher = self.create_publisher(MarkerArray, '/table/grid',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'world', config['frame']
        transform.header.stamp = self.get_clock().now().to_msg()
        for axis, value in zip('xyz',config['world_from_table']['translation_m']):
            setattr(transform.transform.translation, axis, float(value))
        yaw = math.radians(config['world_from_table']['yaw_deg'])/2
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(yaw), math.cos(yaw)
        self.broadcaster.sendTransform(transform)
        self.timer = self.create_timer(1., self.publish)
        self.publish()
        self.get_logger().info('Table grid A1..X16, 384 holes, 50 mm pitch; nominal placement and unverified 25 mm offsets')

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.markers)


def main():
    lock = open(f'/tmp/jaka-table-grid-{os.getuid()}-{os.environ.get("ROS_DOMAIN_ID", "0")}.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('Table grid is already running in this ROS domain.', flush=True)
        return
    rclpy.init()
    node = TableGrid()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
