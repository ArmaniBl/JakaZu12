"""Read-only camera/point-cloud acquisition check; no robot commands."""
from datetime import datetime, timezone
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec/1e9


def cloud_xyz(message):
    fields = {f.name:f for f in message.fields}
    formats = {PointField.FLOAT32:'f4', PointField.FLOAT64:'f8'}
    endian = '>' if message.is_bigendian else '<'
    for name in ('x','y','z'):
        if name not in fields or fields[name].datatype not in formats or fields[name].count != 1:
            raise ValueError('Unsupported XYZ point fields')
    dtype = np.dtype({'names':['x','y','z'],
                      'formats':[endian+formats[fields[name].datatype] for name in ('x','y','z')],
                      'offsets':[fields[name].offset for name in ('x','y','z')],
                      'itemsize':message.point_step})
    points = np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                        strides=(message.row_step,message.point_step))
    return np.stack([points[name].ravel() for name in ('x','y','z')],axis=-1)


parser=argparse.ArgumentParser()
parser.add_argument('--reliable',action='store_true')
parser.add_argument('--cloud-only',action='store_true')
parser.add_argument('--seconds',type=float,default=18)
args=parser.parse_args()
rclpy.init()
node = Node('welding_camera_diagnostics')
buffer = Buffer()
listener = TransformListener(buffer,node)
messages, arrivals, stamps, subscriptions = {}, {}, {}, []
cloud_lags=[]
wall_lags=[]
types = {'/depth_pcl':PointCloud2,'/depth':Image,'/rgb':Image,'/camera_info':CameraInfo,'/clock':Clock}
if args.cloud_only:types={topic:types[topic] for topic in ('/depth_pcl','/camera_info','/clock')}
def receive(topic,message):
    messages[topic] = message
    arrivals.setdefault(topic,[]).append(time.monotonic())
    if hasattr(message,'header'):
        stamps.setdefault(topic,[]).append(stamp(message))
    if topic=='/depth_pcl' and '/clock' in messages:
        clock=messages['/clock'].clock
        lag=clock.sec+clock.nanosec/1e9-stamp(message)
        if abs(lag)<86400:cloud_lags.append(lag)
    if topic=='/depth_pcl':
        wall_lags.append(time.time()-stamp(message))
for topic, msg_type in types.items():
    subscriptions.append(node.create_subscription(msg_type,topic,
        lambda message, topic=topic:receive(topic,message),
        QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE) if args.reliable else qos_profile_sensor_data))
started=time.monotonic()
deadline = started+args.seconds
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.1)
finished=time.monotonic()
report = {'utc':datetime.now(timezone.utc).isoformat(),'duration_sec':args.seconds,
          'reliable':args.reliable,'topics':{},'tf_frames':buffer.all_frames_as_yaml()}
for topic in types:
    msg=messages.get(topic)
    times=arrivals.get(topic,[])
    meta={'messages':len(times),'arrival_hz':(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else None,
          'messages_per_observation_second':len(times)/(finished-started),
          'first_message_after_sec':times[0]-started if times else None,
          'last_message_age_sec':finished-times[-1] if times else None,
          'max_interarrival_sec':max(np.diff(times)).item() if len(times)>1 else None,
          'messages_in_5sec_bins':np.histogram(np.array(times)-started,bins=np.arange(0,args.seconds+5,5))[0].tolist(),
          'publishers':[{'node':p.node_name,'reliability':str(p.qos_profile.reliability)} for p in node.get_publishers_info_by_topic(topic)]}
    if msg is not None and hasattr(msg,'header'):
        meta.update(frame=msg.header.frame_id,stamp=stamp(msg),system_time_delta_sec=time.time()-stamp(msg))
    if len(stamps.get(topic,[]))>1:
        delta=np.diff(stamps[topic]);meta['non_increasing_stamps']=int(np.count_nonzero(delta<=0))
        meta['stamp_hz']=float(1/np.median(delta[delta>0])) if np.any(delta>0) else None
    report['topics'][topic]=meta
if cloud_lags:
    report['cloud_clock_lag_at_receipt_percentiles_sec']=np.percentile(cloud_lags,[0,50,100]).tolist()
if wall_lags:
    report['cloud_system_time_lag_at_receipt_percentiles_sec']=np.percentile(wall_lags,[0,50,100]).tolist()
root=Path(__file__).resolve().parents[1]/'diagnostics'/('camera_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
root.mkdir(parents=True)
cloud=messages.get('/depth_pcl')
if cloud is not None:
    meta=report['topics']['/depth_pcl']
    meta.update(width=cloud.width,height=cloud.height,point_step=cloud.point_step,row_step=cloud.row_step,
                data_bytes=len(cloud.data),fields=[{'name':f.name,'offset':f.offset,'datatype':f.datatype,'count':f.count} for f in cloud.fields])
    try:
        xyz=cloud_xyz(cloud)
        finite=xyz[np.all(np.isfinite(xyz),axis=1)]
        meta.update(total_points=len(xyz),finite_points=len(finite),invalid_points=len(xyz)-len(finite))
        if len(finite):
            meta.update(min_xyz=finite.min(axis=0).tolist(),max_xyz=finite.max(axis=0).tolist(),
                        percentiles_xyz=np.percentile(finite,[1,50,99],axis=0).tolist(),
                        zero_points=int(np.count_nonzero(np.all(finite==0,axis=1))))
            np.savez_compressed(root/'cloud.npz',xyz=xyz,frame=np.array(cloud.header.frame_id))
        meta['world_tf_latest_available']=buffer.can_transform('world',cloud.header.frame_id,rclpy.time.Time())
        meta['world_tf_at_cloud_stamp_available']=buffer.can_transform('world',cloud.header.frame_id,rclpy.time.Time.from_msg(cloud.header.stamp))
        if meta['world_tf_at_cloud_stamp_available']:
            transform=buffer.lookup_transform('world',cloud.header.frame_id,rclpy.time.Time.from_msg(cloud.header.stamp)).transform
            q=transform.rotation
            rotation=np.array([[1-2*(q.y*q.y+q.z*q.z),2*(q.x*q.y-q.z*q.w),2*(q.x*q.z+q.y*q.w)],
                               [2*(q.x*q.y+q.z*q.w),1-2*(q.x*q.x+q.z*q.z),2*(q.y*q.z-q.x*q.w)],
                               [2*(q.x*q.z-q.y*q.w),2*(q.y*q.z+q.x*q.w),1-2*(q.x*q.x+q.y*q.y)]])
            t=transform.translation
            world=finite@rotation.T+np.array([t.x,t.y,t.z])
            top=world[np.abs(world[:,2]-.2)<.002]
            meta['workpiece_top_world']={'points':len(top),'min_xyz':top.min(axis=0).tolist() if len(top) else None,
                                         'max_xyz':top.max(axis=0).tolist() if len(top) else None}
    except Exception as error:
        meta['error']=str(error)
info=messages.get('/camera_info')
if info is not None:
    report['topics']['/camera_info'].update(width=info.width,height=info.height,k=list(info.k),
        p=list(info.p),distortion_model=info.distortion_model,d=list(info.d))
for topic in ('/depth','/rgb'):
    msg=messages.get(topic)
    if msg is not None:
        report['topics'][topic].update(width=msg.width,height=msg.height,encoding=msg.encoding,step=msg.step,data_bytes=len(msg.data))
        if topic=='/depth' and msg.encoding in ('32FC1','16UC1'):
            code=('>' if msg.is_bigendian else '<')+('f4' if msg.encoding=='32FC1' else 'u2')
            values=np.ndarray((msg.height,msg.width),dtype=code,buffer=msg.data,strides=(msg.step,np.dtype(code).itemsize))
            valid=values[np.isfinite(values)&(values>0)]
            if len(valid):report['topics'][topic]['positive_value_percentiles']=np.percentile(valid,[1,50,99]).tolist()
clock=messages.get('/clock')
if clock is not None:
    clock_time=clock.clock.sec+clock.clock.nanosec/1e9
    report['clock_time']=clock_time
    if cloud is not None:
        report['cloud_and_clock_share_time_domain']=abs(stamp(cloud)-clock_time)<86400
        if report['cloud_and_clock_share_time_domain']:report['cloud_minus_clock_sec']=stamp(cloud)-clock_time
report['tf_publishers']={topic:len(node.get_publishers_info_by_topic(topic)) for topic in ('/tf','/tf_static')}
(root/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
print('Saved:',root)
node.destroy_node()
rclpy.shutdown()
