"""Acquisition sequencing without cameras, projector, robot or ROS."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from scan_stream import CaptureLoop


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
