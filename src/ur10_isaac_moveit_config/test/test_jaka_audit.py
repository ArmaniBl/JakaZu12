"""Regression checks for the recorded tabletop incident and the status stream."""
import subprocess
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Duration
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from welding_sim.environment import simulation_floor
from welding_sim.jaka_feedback import FeedbackCapture, packets
from welding_sim.profiles import JAKA
from welding_sim.table_clearance import TableClearance

ROOT = Path(__file__).resolve().parents[3]


class JakaAuditTests(unittest.TestCase):
    def test_table_clearance_reproduces_recorded_wrist_descent(self):
        description = subprocess.check_output([
            'xacro', str(ROOT/'src/jaka_zu12_moveit_config/config/jaka_zu12.urdf.xacro')])
        floor = simulation_floor(.005)
        floor.primitives[0].dimensions = [.8, 1.2, .1]
        joints = list(JAKA.joints)
        q = [1.995787, 2.231310, 2.020064, .586854, -3.874049, 3.363700]
        initial = JointState(name=joints, position=q)
        path = JointTrajectory(joint_names=joints, points=[JointTrajectoryPoint(positions=q)])
        model = TableClearance(description, JAKA, floor)
        report = model.minimum([('descent', RobotTrajectory(joint_trajectory=path))], initial)
        self.assertEqual((report['link'], report['obstacle'], report['stage']),
                         ('Link_5', 'floor', 'descent'))
        self.assertAlmostEqual(report['clearance_m'], -.000126, places=6)

    def test_status_stream_handles_tcp_boundaries_and_reports_faults(self):
        connection = Mock()
        connection.recv.side_effect = [
            b'{"joint_actual_position":[0,0,0,0,0,1],',
            b'"joint_position":[0,0,0,0,0,0]}\x00{"errcode":"0x42","protective_stop":1}', b'']
        frames = list(packets(connection))
        with patch('welding_sim.jaka_feedback.threading.Thread') as thread:
            capture = FeedbackCapture('unused')
            path = JointTrajectory(joint_names=list(JAKA.joints), points=[
                JointTrajectoryPoint(positions=[0.] * 6, time_from_start=Duration(sec=2))])
            capture.begin('lift', path)
            for frame in frames:
                capture.record(frame)
            capture.end(dict(status='timed_out', cancel_acknowledged=True))
            capture.close()
            thread.return_value.join.assert_called_once()
        stage = capture.report['stages'][0]
        self.assertEqual(stage['status'], 'timed_out')
        self.assertTrue(stage['cancel_acknowledged'])
        self.assertEqual(stage['max_joint_error_controller_units'], 1)
        self.assertEqual(stage['samples'][-1]['errcode'], '0x42')
        self.assertEqual(stage['samples'][-1]['protective_stop'], 1)
        connection.sendall.assert_not_called()
