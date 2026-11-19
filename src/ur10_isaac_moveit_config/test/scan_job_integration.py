"""Direct-run isolated integration test: fake scanner -> ROS scene -> approval."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatusArray, GoalStatus
from moveit_msgs.action import ExecuteTrajectory
from geometry_msgs.msg import Pose, Point, PoseStamped
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene, GetPositionFK
from welding_sim.scan_model import sample_object
from welding_sim.scan_alignment import transform_points


def main():
    os.environ['ROS_DOMAIN_ID']='184';os.environ['ROS_LOCALHOST_ONLY']='1'
    rclpy.init();node=Node('scan_job_test_scene')
    vertices=np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                       [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]],float)*.075
    triangles=[[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
               [3,7,6],[3,6,2],[0,4,7],[0,7,3],[1,2,6],[1,6,5]]
    obj=CollisionObject(id='welding_workpiece');obj.header.frame_id='world'
    obj.pose.orientation.w=1.;obj.pose.position.y=-.6;obj.pose.position.z=.075
    pose=Pose();pose.orientation.w=1.
    obj.meshes=[Mesh(vertices=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in vertices],
                     triangles=[MeshTriangle(vertex_indices=t) for t in triangles])];obj.mesh_poses=[pose]
    model,normals=sample_object(obj,spacing=.002)
    center=model.mean(0);correction=np.eye(4)
    correction[:3,:3]=Rotation.from_euler('z',4,degrees=True).as_matrix()
    correction[:3,3]=center+np.array([.012,-.008,.004])-correction[:3,:3]@center
    cloud=transform_points(model[np.sum(normals,axis=1)>.5][::4],correction)
    nominal=np.eye(4);nominal[:3,3]=[0,-600,75]
    bundle=dict(schema='kvantron-moveit/v1',units='mm',frame_signature='fixture-frame',
                camera_T_cad_mm=nominal.tolist(),vertices_mm=(vertices*1000).tolist(),triangles=triangles,
                camera_points_mm=(cloud*1000).tolist(),identity='capture-one',capture_id='capture-one',part_name='fixture')
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            raw=json.dumps(dict(bundle,created_unix=time.time())).encode()
            self.send_response(200);self.end_headers();self.wfile.write(raw)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    floor=CollisionObject(id='floor');floor.header.frame_id='world';floor.pose.orientation.w=1.
    p=Pose();p.orientation.w=1.;p.position.x=-.09;p.position.y=-.465;p.position.z=-.05
    floor.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.8,1.2,.1])];floor.primitive_poses=[p]
    scene=PlanningScene();scene.world.collision_objects=[floor,obj];applied=[]
    def get(req,res):res.scene=scene;return res
    def apply(req,res):
        applied.extend(req.scene.world.collision_objects)
        scene.world.collision_objects=[floor]+list(req.scene.world.collision_objects)
        res.success=True;return res
    node.create_service(GetPlanningScene,'/get_planning_scene',get)
    node.create_service(ApplyPlanningScene,'/apply_planning_scene',apply)
    def fk(req,res):
        assert req.fk_link_names==['welding_tcp']
        assert req.robot_state.joint_state.name==[f'joint_{i}' for i in range(1,7)]
        pose=PoseStamped();pose.header.frame_id=req.header.frame_id
        pose.pose.position.x=.12;pose.pose.position.y=-.55;pose.pose.position.z=.23;pose.pose.orientation.w=1.
        res.pose_stamped=[pose];res.fk_link_names=req.fk_link_names;res.error_code.val=1;return res
    node.create_service(GetPositionFK,'/compute_fk',fk)
    execution=ActionServer(node,ExecuteTrajectory,'/execute_trajectory',
                           execute_callback=lambda goal: ExecuteTrajectory.Result())
    status=node.create_publisher(GoalStatusArray,'/execute_trajectory/_action/status',QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    busy={'value':False}
    joints=node.create_publisher(JointState,'/joint_states',10)
    def publish():
        # No empty heartbeat: reproduce a fresh MoveIt server before its first goal.
        if busy['value']:
            status.publish(GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)]))
        joints.publish(JointState(name=[f'joint_{i}' for i in range(1,7)],position=[0.]*6))
    node.create_timer(.008,publish)  # Actual JAKA driver rate: 125 Hz.
    spin=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);spin.start()
    try:
        with tempfile.TemporaryDirectory() as directory:
            tmp=Path(directory);cal=tmp/'cal.json';report=tmp/'report.json'
            cal.write_text(json.dumps(dict(frame='world',fixed_scanner=True,validation_max_error_m=.0002,
                                          frame_signature='fixture-frame',scanner_to_world=np.eye(4).tolist())))
            script=Path(__file__).resolve().parents[1]/'scripts/scan_job.py'
            base=[sys.executable,str(script)]
            options=['--scanner-url',f'http://127.0.0.1:{server.server_port}',
                     '--calibration',str(cal),'--report',str(report),'--pose-dir',str(tmp/'tcp')]
            def command(action,success=True):
                result=subprocess.run(base+[action]+options,capture_output=True,text=True,timeout=45)
                assert (result.returncode==0)==success,result.stdout+result.stderr
                return result
            command('tcp');assert not applied
            snapshots=list((tmp/'tcp').glob('*.json'));assert len(snapshots)==1
            saved=json.loads(snapshots[0].read_text())
            assert saved['position_m']==[.12,-.55,.23]
            assert saved['camera_point'] is None and saved['calibration_pair_complete'] is False
            command('detect');assert not applied
            command('apply');assert len(applied)==1
            target=applied[0].pose.position
            expected=transform_points(np.array([[0,-.6,.075]]),correction)[0]
            np.testing.assert_allclose([target.x,target.y,target.z],expected,atol=.0003)
            assert applied[0].meshes==obj.meshes
            command('check')
            bundle['identity']='capture-two'
            command('check',False)
            assert len(applied)==1
            busy['value']=True
            rejected=command('check',False)
            assert 'MoveIt is executing' in rejected.stderr,rejected.stderr
            assert len(applied)==1
            print('PASS: measured 6D pose -> approved CAD import -> current receipt -> changed scan blocked; no robot driver')
    finally:
        server.shutdown();rclpy.shutdown();spin.join(timeout=2)


if __name__=='__main__':main()
