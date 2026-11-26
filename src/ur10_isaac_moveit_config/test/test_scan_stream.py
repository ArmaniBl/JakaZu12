"""Acquisition sequencing without cameras, projector, robot or ROS."""
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from unittest.mock import patch
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from scan_stream import CaptureLoop
from welding_sim.operation_lock import welding_operation, welding_operation_active


class Scanner:
    def __init__(self):
        self.starts = 0
        self.complete('old')

    def complete(self, capture):
        self.state = dict(job=dict(state='COMPLETE', capture_id=capture),
                          fresh_capture_for_task=True, active_task={'id': 'task'})

    def request(self, action):
        if action == 'state':
            return self.state
        self.starts += 1
        self.state = dict(job=dict(state='CAPTURING', capture_id=str(self.starts)),
                          active_task={'id': 'task'})
        return self.state


class CaptureLoopTests(unittest.TestCase):
    def test_workpiece_change_waits_for_idle_and_restarts_a_paused_loop(self):
        calls = []
        state = dict(job=dict(state='CAPTURING'),
                     active_task=dict(name='old', part_id='part'), fresh_capture_for_task=True)

        def request(action, body=None):
            calls.append((action, body))
            if action == 'capture':
                return dict(job=dict(state='CAPTURING', capture_id='new'))
            return state

        loop = CaptureLoop(request, continuous=True)
        loop.paused = 'Capture limit'
        loop.failures = 3
        fetch = lambda: self.fail('The old task cloud must not be fetched')
        self.assertIsNone(loop.poll(fetch, 'MoveIt-new'))
        self.assertEqual(calls, [('state', None)])
        state['job']['state'] = 'COMPLETE'
        self.assertIsNone(loop.poll(fetch, 'MoveIt-new'))
        self.assertEqual(calls[-2:], [('new-task', dict(name='MoveIt-new', part_id='part')),
                                      ('capture', None)])
        self.assertIsNone(loop.paused)
        self.assertEqual(loop.failures, 0)
        self.assertEqual(loop.applied_restart, 'MoveIt-new')

    def test_reapplying_same_cad_keeps_inflight_scan_for_export(self):
        scanner = Scanner()
        loop = CaptureLoop(scanner.request, continuous=True)
        task = dict(id='task', name='MoveIt-04-pipe-plate-' + 'a' * 32, capture_count=5)
        restart = 'MoveIt-04-pipe-plate-' + 'b' * 32
        scanner.state = dict(job=dict(state='DECODING'), active_task=task)
        self.assertIsNone(loop.poll(lambda: self.fail('Still decoding'), restart))
        self.assertEqual(loop.applied_restart, restart)
        scanner.complete('fresh')
        scanner.state['active_task'] = dict(task, capture_id='fresh')
        self.assertEqual(loop.poll(lambda: dict(task_id='task', capture_id='fresh'))['capture_id'], 'fresh')
        self.assertEqual(scanner.starts, 1)

    def test_replaced_capture_cannot_reappear(self):
        scanner=Scanner();loop=CaptureLoop(scanner.request)
        def accept(capture):
            scanner.complete(capture)
            scanner.state['active_task']['capture_id']=capture
            return loop.poll(lambda:dict(task_id='task',capture_id=capture))
        self.assertEqual(accept('old')['capture_id'],'old')
        self.assertEqual(accept('new')['capture_id'],'new')
        with self.assertRaisesRegex(ValueError,'Outdated'):accept('old')
        self.assertEqual(loop.displayed_capture,'new')

    def test_task_switch_between_state_and_export_is_rejected(self):
        scanner=Scanner();loop=CaptureLoop(scanner.request)
        with self.assertRaisesRegex(ValueError,'Outdated'):
            loop.poll(lambda:dict(task_id='another-task',capture_id='other'))
    def test_complete_then_next_capture_and_no_overlap(self):
        scanner = Scanner()
        loop = CaptureLoop(scanner.request, continuous=True)
        self.assertEqual(loop.poll(lambda: 'old cloud'), 'old cloud')
        self.assertEqual(scanner.starts, 1)
        self.assertIsNone(loop.poll(lambda: self.fail('Must not export during capture')))
        self.assertEqual(scanner.starts, 1)
        scanner.complete('1')
        self.assertEqual(loop.poll(lambda: 'new cloud'), 'new cloud')
        self.assertEqual(scanner.starts, 2)

    def test_motion_cancels_owned_capture_and_resumes_once_at_end(self):
        scanner = Scanner()
        cancellations = []
        def request(action):
            if action == 'cancel':
                cancellations.append(scanner.state['job']['capture_id'])
                return scanner.state
            return scanner.request(action)

        loop = CaptureLoop(request, continuous=True)
        loop.poll(lambda: None)
        for _ in range(3):
            loop.poll(lambda: self.fail('Must not export during motion'), moving=lambda: True)
            self.assertEqual(loop.state, 'PAUSED_MOTION')
        self.assertEqual(cancellations, ['1'])
        self.assertEqual(scanner.starts, 1)
        # A capture may complete before the cancellation reaches the scanner.
        scanner.complete('1')
        scanner.state['active_task']['capture_id'] = '1'
        loop.poll(lambda: self.fail('Must not publish a scan interrupted by motion'))
        self.assertEqual(scanner.starts, 2)
        self.assertIsNone(loop.paused)

    def test_operator_cancel_during_idle_stays_paused_after_an_operation(self):
        scanner = Scanner()
        loop = CaptureLoop(scanner.request, continuous=True)
        loop.poll(lambda: None)
        scanner.state['job']['state'] = 'CANCELLED'
        loop.poll(lambda: None)
        loop.poll(lambda: None, moving=lambda: True)
        loop.poll(lambda: None)
        self.assertEqual(scanner.starts, 1)
        self.assertTrue(loop.paused)

    def test_operation_lock_covers_stage_gaps_and_releases_on_failure(self):
        with TemporaryDirectory() as directory:
            with patch('welding_sim.operation_lock._path', return_value=Path(directory)/'operation.lock'):
                self.assertFalse(welding_operation_active())
                with self.assertRaisesRegex(ValueError, 'stage failed'):
                    with welding_operation():
                        self.assertTrue(welding_operation_active())
                        with self.assertRaisesRegex(RuntimeError, 'operation is already running'):
                            with welding_operation():
                                self.fail('Overlapping job')
                        raise ValueError('stage failed')
                self.assertFalse(welding_operation_active())

    def test_table_reconstruction_reuses_completed_frames_after_legacy_failure(self):
        scanner = Scanner()
        loop = CaptureLoop(scanner.request, continuous=True)
        exports = lambda: self.fail('Do not fetch a legacy export with tabletop stereo')
        reconstruct = lambda capture, task: dict(capture_id=capture, task_id=task)
        self.assertIsNone(loop.poll(exports, reconstruct=reconstruct))
        scanner.state['job'].update(state='FAILED', reason='legacy geometry rejected')
        result = loop.poll(exports, reconstruct=reconstruct)
        self.assertEqual(result['capture_id'], '1')
        self.assertEqual(loop.failures, 0)
        self.assertEqual(scanner.starts, 2)

    def test_table_cloud_is_published_before_legacy_worker_finishes(self):
        scanner = Scanner()
        loop = CaptureLoop(scanner.request, continuous=True)
        calls = []

        def reconstruct(capture, task):
            calls.append(capture)
            return dict(capture_id=capture, task_id=task)

        loop.poll(lambda: None, reconstruct=reconstruct)
        scanner.state['job']['state'] = 'DECODING'
        self.assertEqual(loop.poll(lambda: None, reconstruct=reconstruct)['capture_id'], '1')
        self.assertEqual(scanner.starts, 1)
        self.assertIsNone(loop.poll(lambda: None, reconstruct=reconstruct))
        scanner.state['job'].update(state='FAILED', reason='legacy geometry rejected')
        self.assertIsNone(loop.poll(lambda: None, reconstruct=reconstruct))
        self.assertEqual(calls, ['1'])
        self.assertEqual(scanner.starts, 2)
        self.assertIsNone(loop.last_failure)

    def test_operator_cancel_stays_paused(self):
        scanner = Scanner()
        loop = CaptureLoop(scanner.request, continuous=True)
        loop.poll(lambda: None)
        scanner.state['job']['state'] = 'CANCELLED'
        loop.poll(lambda: None)
        loop.poll(lambda: None)
        self.assertTrue(loop.paused)
        self.assertEqual(scanner.starts, 1)

    def test_failed_scans_retry_at_most_three_times(self):
        scanner = Scanner()
        loop = CaptureLoop(scanner.request, continuous=True)
        loop.poll(lambda: None)
        for capture in ('1', '2', '3'):
            scanner.state['job'] = dict(state='FAILED', capture_id=capture, reason='motion')
            loop.poll(lambda: None)
        self.assertTrue(loop.paused)
        self.assertEqual(scanner.starts, 3)

    def test_view_only_never_starts_capture(self):
        scanner = Scanner()
        loop = CaptureLoop(scanner.request)
        self.assertEqual(loop.poll(lambda: 'cloud'), 'cloud')
        self.assertEqual(scanner.starts, 0)

    def test_capture_rejection_keeps_completed_cloud(self):
        scanner = Scanner()
        def request(action):
            if action == 'capture':
                raise ValueError('Camera already in use')
            return scanner.state
        loop = CaptureLoop(request, continuous=True)
        self.assertEqual(loop.poll(lambda: 'cloud'), 'cloud')
        self.assertIn('Camera already in use', loop.paused)


if __name__ == '__main__':
    unittest.main()
