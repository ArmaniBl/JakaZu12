"""Real rclpy futures: cancellation must free the monitor's request slot."""
import unittest
from rclpy.task import Future
from welding_sim.web_server import pending_request

class MonitorRecoveryTests(unittest.TestCase):
    def test_timeout_drops_cancelled_future_even_when_not_done(self):
        future=Future()
        self.assertIsNone(pending_request(future,0.,4.))
        self.assertTrue(future.cancelled())
        self.assertFalse(future.done())
        self.assertIsNone(pending_request(future,0.,5.))
        replacement=Future()
        self.assertIs(pending_request(replacement,5.,5.1),replacement)

    def test_success_and_fresh_pending_requests_are_kept(self):
        future=Future()
        self.assertIs(pending_request(future,1.,2.),future)
        future.set_result('scene')
        self.assertIs(pending_request(future,1.,20.),future)
        self.assertEqual(future.result(),'scene')
        self.assertIsNone(pending_request(None,0.,100.))

if __name__=='__main__':unittest.main()
