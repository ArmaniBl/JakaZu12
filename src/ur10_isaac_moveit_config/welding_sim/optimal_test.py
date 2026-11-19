"""Experimental best-of-candidates planner. Execution is deliberately unavailable."""
import argparse
from copy import deepcopy
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetPositionIK
from rosidl_runtime_py.convert import message_to_ordereddict

from . import runner
from .geometry import rotation
from .orientation import tilted_orientation
from .path_quality import trajectory_quality


def seed_states(state, joints, limits):
    values = dict(zip(state.joint_state.name,state.joint_state.position))
    changes = [{}, {joints[2]:-values[joints[2]]},
               {joints[4]:values[joints[4]]+math.pi,joints[5]:values[joints[5]]-math.pi},
               {joints[4]:values[joints[4]]-math.pi,joints[5]:values[joints[5]]+math.pi}]
    seeds = []
    for change in changes:
        target = values.copy()
        for name, value in change.items():
            low, high = limits[name]
            target[name] = float(np.clip(value, low+.001, high-.001))
        if any(all(abs(target[j]-dict(zip(s.joint_state.name,s.joint_state.position))[j])<1e-6 for j in joints) for s in seeds):
            continue
        seed = deepcopy(state)
        seed.joint_state.position = [target[j] for j in seed.joint_state.name]
        seeds.append(seed)
    return seeds


class OptimalTestRunner(runner.WeldingRunner):
    max_candidates = 12
    search_seconds = 180.

    def __init__(self,args):
        if args.mode != 'plan' or args.robot != 'jaka_real':
            raise ValueError('Experimental launcher supports JAKA planning only; execution is unavailable')
        super().__init__(args)

    def fresh_state(self):
        if getattr(self,'search_start',None) is not None:
            return deepcopy(self.search_start)
        return super().fresh_state()

    def approach(self,state,position,orientation,seed=None):
        seeds = seed_states(state,self.profile.joints,self.seed_limits)
        chosen = seeds[self.seed_index % len(seeds)]
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.profile.group
        request.ik_request.robot_state = chosen
        request.ik_request.ik_link_name = self.profile.tip
        request.ik_request.pose_stamped.header.frame_id = self.profile.frame
        request.ik_request.pose_stamped.pose = runner.pose_message(position,orientation)
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 1
        result = self.call(GetPositionIK,'/compute_ik',request)
        if result.error_code.val != 1:
            raise RuntimeError(f'Candidate IK failed ({result.error_code.val})')
        target = dict(zip(result.solution.joint_state.name,result.solution.joint_state.position))
        current = dict(zip(state.joint_state.name,state.joint_state.position))
        # Pick an equivalent bounded revolute goal close to the current joint.
        # The entire physical route to that goal is still collision-planned.
        for joint in self.profile.joints:
            value = target[joint]
            low,high = self.seed_limits[joint]
            equivalent = [value+k*math.tau for k in range(-2,3) if low<=value+k*math.tau<=high]
            if not equivalent or not math.isfinite(value):
                raise RuntimeError('IK goal violates model limits: '+joint)
            target[joint] = min(equivalent,key=lambda q:abs(q-current[joint]))
            if abs(target[joint]-value)>1e-6:
                self.report.setdefault('ik_goal_normalizations',[]).append(dict(joint=joint,original_rad=value,chosen_rad=target[joint]))
        return self.joint_motion(state,target)

    def execute(self,stages):
        raise RuntimeError('Experimental test cannot execute robot motion')

    def plan(self,paths):
        self.search_start = self.settled_state()
        start = deepcopy(self.search_start)
        # settled_state needs fresh feedback; only freeze after it has returned.
        joints = self.profile.joints
        values = dict(zip(start.joint_state.name,start.joint_state.position))
        model = Path(get_package_share_directory('jaka_description'))/'urdf/jaka_zu12.urdf'
        self.seed_limits = {j.attrib['name']:(float(j.find('limit').attrib['lower']),float(j.find('limit').attrib['upper']))
                            for j in ET.parse(model).getroot().findall('joint') if j.attrib.get('name') in joints}
        if set(self.seed_limits) != set(joints):
            raise RuntimeError('Cannot read JAKA seed limits')
        request = GetPositionFK.Request()
        request.header.frame_id = self.profile.frame
        request.fk_link_names = [self.profile.tip]
        request.robot_state = start
        fk = self.call(GetPositionFK,'/compute_fk',request)
        if fk.error_code.val != 1 or len(fk.pose_stamped)!=1:
            raise RuntimeError('Cannot determine current tool orientation')
        q = fk.pose_stamped[0].pose.orientation
        current_rotation = rotation([q.x,q.y,q.z,q.w])
        nominal = tilted_orientation(paths[0][3],self.args.tilt,self.args.tilt_direction)
        relative = rotation(nominal).T@current_rotation
        nearest_roll = math.degrees(math.atan2(relative[1,0]-relative[0,1],relative[0,0]+relative[1,1]))
        rolls = list(dict.fromkeys(round(v,5) for v in [0.,nearest_roll,nearest_roll+30.,nearest_roll-30.,90.,-90.,180.]))
        if getattr(self.args,'orientation_mode','auto')=='fixed':
            rolls = [0.]
        choices = [(roll,seed) for seed in range(4) for roll in rolls]
        # Baseline first, then local seed/roll changes before more distant variants.
        local_roll = rolls[1] if len(rolls)>1 else 0.
        choices = [(0.,0),(local_roll,0),(0.,1),(local_roll,1)]+choices
        choices = list(dict.fromkeys(choices))[:self.max_candidates]
        original_report = deepcopy(self.report)
        publisher = self.display
        self.display = SimpleNamespace(publish=lambda message:None)
        self.orientation_deadline = None
        attempts = []
        best = None
        started = time.monotonic()
        try:
            for index,(roll,seed_index) in enumerate(choices):
                if time.monotonic()-started>=self.search_seconds:
                    break
                self.seed_index = seed_index
                self.report = deepcopy(original_report)
                self.cartesian_targets,self.joint_targets = {},{}
                candidate = [(edge,seam,points,tilted_orientation(
                                 tilted_orientation(q,self.args.tilt,self.args.tilt_direction),0.,0.,roll))
                             for edge,seam,points,q in paths]
                record = dict(index=index,roll_delta_deg=roll,ik_seed=seed_index)
                self.get_logger().info(f'Optimal test candidate {index+1}/{len(choices)}: roll={roll:g}, seed={seed_index}')
                try:
                    stages = runner.WeldingRunner.plan_fixed(self,candidate)
                    quality = trajectory_quality(stages,values,joints)
                    record.update(status='planned',quality=quality)
                    self.get_logger().info(f"Candidate {index+1}: cost={quality['cost']:.3f}, duration={quality['duration_sec']:.1f}s")
                    if best is None or quality['cost']<best['quality']['cost']:
                        best = dict(stages=stages,quality=quality,report=deepcopy(self.report),paths=candidate,
                                    cartesian=deepcopy(self.cartesian_targets),joint=deepcopy(self.joint_targets),index=index)
                except (RuntimeError,ValueError) as error:
                    record.update(status='failed',error=str(error))
                attempts.append(record)
                self.report = deepcopy(original_report)
                self.report['optimization'] = dict(attempts=deepcopy(attempts),execution_enabled=False)
        finally:
            self.display = publisher
            self.search_start = None
        if best is None:
            raise RuntimeError('No complete collision-free candidate; inspect optimization attempts')
        actual = super().fresh_state()
        actual_values = dict(zip(actual.joint_state.name,actual.joint_state.position))
        if max(abs(actual_values[j]-values[j]) for j in joints)>.005:
            raise RuntimeError('Robot moved during comparison; no selected plan published')
        scene = self.call(GetPlanningScene,'/get_planning_scene',GetPlanningScene.Request()).scene
        if (sorted(scene.world.collision_objects,key=lambda o:o.id)!=sorted(self.initial_scene.world.collision_objects,key=lambda o:o.id)
                or scene.robot_state.attached_collision_objects!=self.initial_scene.robot_state.attached_collision_objects):
            raise RuntimeError('Collision geometry changed during comparison; plan again')
        self.report = best['report']
        self.planned_paths = best['paths']
        self.cartesian_targets,self.joint_targets = best['cartesian'],best['joint']
        for i,(_,_,_,q) in enumerate(best['paths']):
            if i<len(self.report.get('selection',[])):
                self.report['selection'][i]['orientation_xyzw'] = q.tolist()
        self.report['optimization'] = dict(attempts=attempts,chosen_index=best['index'],quality=best['quality'],
            elapsed_sec=time.monotonic()-started,execution_enabled=False,
            fixed_tilt_deg=self.args.tilt,fixed_tilt_direction_deg=self.args.tilt_direction,
            scope='best complete route among sampled candidates; no global optimality guarantee')
        baseline = next(a for a in attempts if a['status']=='planned')
        self.report['optimization']['baseline_index'] = baseline['index']
        self.report['optimization']['cost_reduction_percent'] = 100*(baseline['quality']['cost']-best['quality']['cost'])/max(baseline['quality']['cost'],1e-12)
        chosen = attempts[best['index']]
        self.report['chosen_orientation'] = dict(tilt_deg=self.args.tilt,direction_deg=self.args.tilt_direction,
                                                roll_delta_deg=chosen['roll_delta_deg'],status='planned')
        self.report['optimized_trajectory'] = [dict(name=name,trajectory=message_to_ordereddict(t)) for name,t in best['stages']]
        self.report['optimization_start_state'] = message_to_ordereddict(start)
        self.display.publish(DisplayTrajectory(model_id=self.profile.model,trajectory_start=start,
                                               trajectory=[t for _,t in best['stages']]))
        self.get_logger().info(f"Selected candidate {best['index']+1}, cost {best['quality']['cost']:.3f}; no motion requested")
        return best['stages']


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False,
        description='Experimental JAKA planning only: compare complete collision-free routes; no robot motion.',
        epilog='Also accepts welding options: --face z_plus, --edge ID, --begin, --end, --tilt, --roll, --speed, --scaling, --object. Start run_real.sh separately. Search time is checked between candidates.')
    parser.add_argument('--max-candidates',type=int,default=12)
    parser.add_argument('--search-seconds',type=float,default=180.)
    options,remaining = parser.parse_known_args()
    if not 2<=options.max_candidates<=40 or not math.isfinite(options.search_seconds) or not 10<=options.search_seconds<=600:
        parser.error('Use 2..40 candidates and 10..600 search seconds')
    OptimalTestRunner.max_candidates,OptimalTestRunner.search_seconds = options.max_candidates,options.search_seconds
    sys.argv = [sys.argv[0],'--robot','jaka_real','--output','diagnostics/optimal_test']+remaining+['--mode','plan']
    return runner.main(runner_type=OptimalTestRunner)
