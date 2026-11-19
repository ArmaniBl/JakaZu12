"""Run directly with the RangeVision venv; fake ROS scene in domain 183 only."""
import json, os, subprocess, tempfile, threading
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from moveit_msgs.action import ExecuteTrajectory
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatusArray
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from welding_sim.scan_model import sample_object

def main():
    os.environ["ROS_DOMAIN_ID"]="183"
    os.environ["ROS_LOCALHOST_ONLY"]="1"
    rclpy.init()
    node=Node('rangevision_test_scene')
    obj=CollisionObject(id='welding_workpiece');obj.header.frame_id='world';obj.pose.orientation.w=1.
    p=Pose();p.orientation.w=1.;p.position.y=-.6;p.position.z=.1
    obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.3,.2,.2])];obj.primitive_poses=[p]
    scene=PlanningScene();scene.world.collision_objects=[obj]
    applied=[]
    def get(req,res):res.scene=scene;return res
    def apply(req,res):
        applied.extend(req.scene.world.collision_objects)
        scene.world.collision_objects=req.scene.world.collision_objects
        res.success=True
        return res
    node.create_service(GetPlanningScene,'/get_planning_scene',get)
    node.create_service(ApplyPlanningScene,'/apply_planning_scene',apply)
    execution=ActionServer(node,ExecuteTrajectory,'/execute_trajectory',
                           execute_callback=lambda goal: ExecuteTrajectory.Result())
    status=node.create_publisher(GoalStatusArray,'/execute_trajectory/_action/status',QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    joints=node.create_publisher(JointState,'/joint_states',10)
    def publish():
        status.publish(GoalStatusArray())
        joints.publish(JointState(name=[f'joint_{i}' for i in range(1,7)],position=[0.]*6))
    node.create_timer(.05,publish)
    thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);thread.start()
    try:
      with tempfile.TemporaryDirectory() as directory:
        root=Path(directory)
        points,normals=sample_object(obj,spacing=.002)
        delta=np.array([.012,-.009,.006])
        points=points[np.sum(normals,axis=1)>.5][::3]+delta
        cloud=root/'scan.xyz';np.savetxt(cloud,points*1000)
        cal=root/'calibration.json';cal.write_text(json.dumps(dict(frame='world',fixed_scanner=True,validation_max_error_m=.0002,scanner_to_world=np.eye(4).tolist())))
        report=root/'report.json'
        cmd=['./run_rangevision.sh','compare','--cloud',str(cloud),'--units','mm','--calibration',str(cal),'--roi','-.2','.2','-.75','-.45','.005','.25','--report',str(report)]
        run=subprocess.run(cmd,capture_output=True,text=True,timeout=25)
        assert run.returncode==0,run.stderr+run.stdout
        assert not applied,'compare modified scene'
        result=json.loads(report.read_text())
        np.testing.assert_allclose(result['translation_m'],delta,atol=.0005)
        run=subprocess.run(['./run_rangevision.sh','apply','--report',str(report)],capture_output=True,text=True,timeout=25)
        assert run.returncode==0,run.stderr+run.stdout
        assert len(applied)==1
        actual=applied[0].pose.position
        np.testing.assert_allclose([actual.x,actual.y,actual.z],delta,atol=.0005)
        assert applied[0].primitives==obj.primitives
        assert applied[0].primitive_poses==obj.primitive_poses
        run=subprocess.run(['./run_rangevision.sh','apply','--report',str(report)],capture_output=True,text=True,timeout=25)
        assert run.returncode!=0,'stale report accepted'
        assert len(applied)==1
        print('PASS: isolated ROS compare -> explicit apply; geometry retained; stale report rejected; no robot driver.')
    finally:
      rclpy.shutdown();thread.join(timeout=2)

if __name__ == "__main__":
    main()
