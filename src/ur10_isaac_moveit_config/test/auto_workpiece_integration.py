"""Isolated ROS test of startup mesh -> measured pose -> idle-only scene update."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatusArray,GoalStatus
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import PlanningScene,CollisionObject
from moveit_msgs.srv import GetPlanningScene,ApplyPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation
from welding_sim.part_preview_fit import top_surface
from welding_sim.workpiece_model import preview_object,PREVIEW_TYPE


def main():
    os.environ.update(ROS_DOMAIN_ID='188',ROS_LOCALHOST_ONLY='1')
    rclpy.init();node=Node('auto_cad_fixture')
    root=Path(__file__).resolve().parents[3]
    model=json.loads((root/'src/jaka_zu12_moveit_config/config/workpiece_model.json').read_text())
    scene=PlanningScene();floor=CollisionObject(id='floor');floor.header.frame_id='world';floor.pose.orientation.w=1.
    pose=Pose();pose.orientation.w=1.;pose.position.x=-.04;pose.position.y=-.465;pose.position.z=-.05
    floor.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.8,1.2,.1])];floor.primitive_poses=[pose]
    scene.world.collision_objects=[floor,preview_object(model)]
    applied=[];busy={'value':False};goals=[]
    def get(request,response):response.scene=scene;return response
    def apply(request,response):
        assert not busy['value']
        assert [o.id for o in request.scene.world.collision_objects]==['welding_workpiece']
        applied.append(deepcopy(request.scene.world.collision_objects[0]))
        scene.world.collision_objects=[floor]+list(request.scene.world.collision_objects)
        response.success=True;return response
    node.create_service(GetPlanningScene,'/get_planning_scene',get)
    node.create_service(ApplyPlanningScene,'/apply_planning_scene',apply)
    def execute(goal):goals.append(goal);raise AssertionError('No motion is permitted in this test')
    execution=ActionServer(node,ExecuteTrajectory,'/execute_trajectory',execute_callback=execute)
    status=node.create_publisher(GoalStatusArray,'/execute_trajectory/_action/status',QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    joints=node.create_publisher(JointState,'/joint_states',10)
    def publish():
        statuses=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)] if busy['value'] else []
        status.publish(GoalStatusArray(status_list=statuses))
        joints.publish(JointState(name=[f'joint_{i}' for i in range(1,7)],position=[0.]*6))
    node.create_timer(.008,publish)
    thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);thread.start()
    try:
        with tempfile.TemporaryDirectory() as directory:
            workspace=Path(directory);(workspace/'src/jaka_zu12_moveit_config/config').mkdir(parents=True)
            _,surface=top_surface(model['vertices_mm'],model['triangles'])
            matrix=Rotation.from_euler('xyz',[2.,-3.,20.],degrees=True).as_matrix();position=np.array([-.3,-.7,.012])
            np.save(workspace/'points.npy',surface@matrix.T+position)
            (workspace/'bundle.json').write_text(json.dumps(dict(model,capture_id='test-capture',frame_signature='test-frame')))
            code='''import sys,json,numpy as np,rclpy,time
from pathlib import Path
sys.path.insert(0,sys.argv[1])
from rangevision import Bridge
from auto_workpiece import AutoWorkpiece
rclpy.init();node=Bridge('auto_fit_integration_client');workspace=Path(sys.argv[2])
auto=AutoWorkpiece(node,workspace);end=time.monotonic()+7
while time.monotonic()<end and not(auto.scene.service_is_ready() and auto.apply.service_is_ready()):rclpy.spin_once(node,timeout_sec=.1)
done=auto.update(json.loads((workspace/'bundle.json').read_text()),np.load(workspace/'points.npy'))
assert auto.result['state']==sys.argv[3],auto.result
assert auto.result['execution_ready'] is False
node.destroy_node();rclpy.try_shutdown()
'''
            command=[sys.executable,'-c',code,str(root/'src/ur10_isaac_moveit_config/scripts'),str(workspace)]
            result=subprocess.run(command+['matched_preview'],capture_output=True,text=True,timeout=35)
            assert result.returncode==0,result.stderr+result.stdout
            assert len(applied)==1 and applied[0].type.key==PREVIEW_TYPE
            actual=applied[0].pose.position
            np.testing.assert_allclose([actual.x,actual.y,actual.z],position,atol=.002)
            busy['value']=True
            result=subprocess.run(command+['waiting_for_idle'],capture_output=True,text=True,timeout=20)
            assert result.returncode==0,result.stderr+result.stdout
            assert len(applied)==1 and not goals
            print('PASS: ROS startup mesh, automatic six-dimensional pose, collision update, busy rejection, no robot goals')
    finally:
        execution.destroy();rclpy.shutdown();thread.join(timeout=3);node.destroy_node()


if __name__=='__main__':main()
