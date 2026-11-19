"""Actual travel, including full wrist turns, determines the test route score."""
import math
from types import SimpleNamespace as NS
import unittest
from welding_sim.path_quality import trajectory_quality

JOINTS = tuple(f'joint_{i}' for i in range(1,7))


def trajectory(rows, seconds=None):
    seconds = seconds or list(range(len(rows)))
    return NS(joint_trajectory=NS(joint_names=list(JOINTS),points=[
        NS(positions=row,time_from_start=NS(sec=t,nanosec=0)) for row,t in zip(rows,seconds)]))


class PathQualityTests(unittest.TestCase):
    def quality(self,rows):
        return trajectory_quality([('approach',trajectory(rows))],dict.fromkeys(JOINTS,0.),JOINTS)

    def test_full_turn_is_not_shortened_modulo_two_pi(self):
        short = self.quality([[0.]*6,[0.]*5+[.1]])
        turn = self.quality([[0.]*6,[0.]*5+[2*math.pi+.1]])
        self.assertGreater(turn['cost'],short['cost']+10.)

    def test_wrist_reversals_cost_more_than_direct_path(self):
        direct = self.quality([[0.]*6,[0.]*5+[.2]])
        detour = self.quality([[0.]*6,[0.]*5+[1.],[0.]*5+[.2]])
        self.assertGreater(detour['cost'],direct['cost'])

    def test_completion_is_included(self):
        start = dict.fromkeys(JOINTS,0.)
        approach = trajectory([[0.]*6,[.1]*6])
        simple = [('approach',approach),('lift',trajectory([[.1]*6,[.2]*6]))]
        detour = [('approach',approach),('lift',trajectory([[.1]*6,[2.]*6,[.2]*6]))]
        self.assertLess(trajectory_quality(simple,start,JOINTS)['cost'],trajectory_quality(detour,start,JOINTS)['cost'])

    def test_discontinuous_stage_is_rejected(self):
        with self.assertRaises(ValueError):
            self.quality([[1.]*6,[2.]*6])

    def test_invalid_timing_is_rejected(self):
        with self.assertRaises(ValueError):
            trajectory_quality([('approach',trajectory([[0.]*6,[.1]*6],[0,0]))],dict.fromkeys(JOINTS,0.),JOINTS)


if __name__ == '__main__':
    unittest.main()
