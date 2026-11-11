"""Repair existing camera in live Isaac Sim; does not control the robot.

Run with Script Editor exec(open(path).read()); start(). Backs up both USD layers.
"""
import asyncio
from datetime import datetime
import json
from pathlib import Path
import traceback

import omni.graph.core as og
import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdRender


def ensure_writer_gate(stage):
    """Isaac 6 NodeWriter executes from WriterSyncGate, bypassing legacy SD gates."""
    helper=stage.GetPrimAtPath('/Graph/ROS_Camera/DepthPclPublish')
    if not helper:
        return
    skip=og.Controller.get(og.Controller.attribute(str(helper.GetPath())+'.inputs:frameSkipCount'))
    product=og.Controller.get(og.Controller.attribute('/Graph/ROS_Camera/RenderProduct.outputs:renderProductPath'))
    prefix='/Render/PostProcess/SDGPipeline/'+str(product).rsplit('/',1)[-1]+'_'
    for prim in list(stage.Traverse()):
        if not str(prim.GetPath()).startswith(prefix):
            continue
        node_type=str(prim.GetAttribute('node:type').Get())
        # Replicator may combine writers in one execution pipeline. Apply the
        # rate once at each writer, keeping upstream acquisition ungated.
        if node_type=='isaacsim.core.nodes.IsaacSimulationGate' and '_WeldingRateGate' not in str(prim.GetPath()):
            with Usd.EditContext(stage,stage.GetSessionLayer()):
                prim.GetAttribute('inputs:step').Set(1)
                og.Controller.set(og.Controller.attribute(str(prim.GetPath())+'.inputs:step'),1)
            continue
        if node_type not in ('isaacsim.ros2.bridge.ROS2PublishPointCloud','isaacsim.ros2.bridge.ROS2PublishCameraInfo'):
            continue
        if prim.GetAttribute('inputs:topicName').Get() not in ('depth_pcl','/depth_pcl','camera_info','/camera_info'):
            continue
        path=str(prim.GetPath())
        gate=path+'_WeldingRateGate'
        target=path+'.inputs:execIn'
        with Usd.EditContext(stage,stage.GetSessionLayer()):
            if not stage.GetPrimAtPath(gate):
                sources=prim.GetAttribute('inputs:execIn').GetConnections()
                if len(sources)!=1:
                    raise RuntimeError('Unexpected cloud execution connections: '+str(sources))
                og.Controller.create_node(gate,'isaacsim.core.nodes.IsaacSimulationGate')
                og.Controller.disconnect(str(sources[0]),target)
                og.Controller.connect(str(sources[0]),gate+'.inputs:execIn')
                og.Controller.connect(gate+'.outputs:execOut',target)
                print('[welding camera repair] Installed camera writer gate:',gate)
            stage.GetPrimAtPath(gate).GetAttribute('inputs:step').Set(int(skip)+1)
            og.Controller.set(og.Controller.attribute(gate+'.inputs:step'),int(skip)+1)


async def repair():
    app=omni.kit.app.get_app()
    for _ in range(8):
        await app.next_update_async()
    stage=omni.usd.get_context().get_stage()
    if stage is None:
        return
    if not stage.GetPrimAtPath('/Graph/ROS_Camera'):
        return
    destination=Path.home()/'.ros'/'welding_camera_repair.json'
    result={'stage':stage.GetRootLayer().identifier}
    try:
        camera='/World/Camera'
        graph='/Graph/ROS_Camera'
        tf_graph='/Graph/WeldingCameraTF'
        if not stage.GetPrimAtPath(camera).IsA(UsdGeom.Camera):
            raise RuntimeError('Missing camera '+camera)
        backup=Path.home()/'.ros'/('camera_backup_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
        backup.mkdir(parents=True)
        stage.GetRootLayer().Export(str(backup/'root.usda'))
        stage.GetSessionLayer().Export(str(backup/'session.usda'))
        result['backup']=str(backup)
        from isaacsim.ros2.nodes.ogn.OgnROS2CameraHelperDatabase import OgnROS2CameraHelperDatabase
        from isaacsim.ros2.nodes.ogn.OgnROS2CameraInfoHelperDatabase import OgnROS2CameraInfoHelperDatabase
        helpers=[('DepthPclPublish',OgnROS2CameraHelperDatabase),('CameraInfoPublish',OgnROS2CameraInfoHelperDatabase)]
        # Writers cache their initial settings. Detach them before reconfiguring.
        for name,database in helpers:
            state=database.per_instance_internal_state(og.Controller.node(f'{graph}/{name}'))
            if state and state.initialized:
                state.custom_reset()
        for _ in range(12):
            await app.next_update_async()
        with Usd.EditContext(stage,stage.GetRootLayer()):
            stage.GetPrimAtPath(graph+'/RenderProduct').GetRelationship('inputs:cameraPrim').SetTargets([Sdf.Path(camera)])
            for name,_ in helpers:
                for key,value in {'useSystemTime':True,'resetSimulationTimeOnStop':False,
                                  'frameSkipCount':11,'queueSize':1,'frameId':'sim_camera'}.items():
                    stage.GetPrimAtPath(f'{graph}/{name}').GetAttribute('inputs:'+key).Set(value)
                    og.Controller.set(og.Controller.attribute(f'{graph}/{name}.inputs:{key}'),value)
            product_path=og.Controller.get(og.Controller.attribute(graph+'/RenderProduct.outputs:renderProductPath'))
            if product_path:
                with Usd.EditContext(stage,stage.GetSessionLayer()):
                    UsdRender.Product(stage.GetPrimAtPath(product_path)).GetCameraRel().SetTargets([Sdf.Path(camera)])
            result['render_product']=str(product_path)
            # USD X right/Y up/-Z forward -> ROS optical X right/Y down/Z forward.
            matrix=UsdGeom.Xformable(stage.GetPrimAtPath(camera)).ComputeLocalToWorldTransform(0)
            optical=Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1,0,0),180))*matrix
            translation=matrix.ExtractTranslation()*UsdGeom.GetStageMetersPerUnit(stage)
            rotation=optical.ExtractRotationQuat()
            quat=[*rotation.GetImaginary(),rotation.GetReal()]
            if stage.GetPrimAtPath(tf_graph):
                if stage.GetPrimAtPath(tf_graph+'/Publish'):
                    og.Controller.delete_node(tf_graph+'/Publish')
            if not stage.GetPrimAtPath(tf_graph):
                k=og.Controller.Keys
                og.Controller.edit({'graph_path':tf_graph,'evaluator_name':'execution'},{
                    k.CREATE_NODES:[('Tick','omni.graph.action.OnPlaybackTick'),
                                    ('Time','isaacsim.core.nodes.IsaacReadSystemTime'),
                                    ('Context','isaacsim.ros2.bridge.ROS2Context'),
                                    ('Publish','isaacsim.ros2.bridge.ROS2PublishRawTransformTree')],
                    k.CONNECT:[('Tick.outputs:tick','Publish.inputs:execIn'),
                               ('Time.outputs:systemTime','Publish.inputs:timeStamp'),
                               ('Context.outputs:context','Publish.inputs:context')]})
            else:
                og.Controller.create_node(tf_graph+'/Publish','isaacsim.ros2.bridge.ROS2PublishRawTransformTree')
                for source,target in [('Tick.outputs:tick','Publish.inputs:execIn'),
                                      ('Time.outputs:systemTime','Publish.inputs:timeStamp'),
                                      ('Context.outputs:context','Publish.inputs:context')]:
                    og.Controller.connect(tf_graph+'/'+source,tf_graph+'/'+target)
            for key,value in {'parentFrameId':'world','childFrameId':'sim_camera',
                              'translation':list(translation),'rotation':quat,
                              'staticPublisher':True,'topicName':'/tf_static'}.items():
                authored=Gf.Quatd(quat[3],Gf.Vec3d(*quat[:3])) if key=='rotation' else Gf.Vec3d(*value) if key=='translation' else value
                stage.GetPrimAtPath(tf_graph+'/Publish').GetAttribute('inputs:'+key).Set(authored)
                og.Controller.set(og.Controller.attribute(f'{tf_graph}/Publish.inputs:{key}'),value)
            result['optical_tf']={'translation':list(translation),'rotation_xyzw':quat}
        # RunOnce already fired: explicitly initialize the replacement writers.
        from isaacsim.ros2.nodes.ogn.python.nodes.OgnROS2CameraHelper import OgnROS2CameraHelper
        from isaacsim.ros2.nodes.ogn.python.nodes.OgnROS2CameraInfoHelper import OgnROS2CameraInfoHelper
        for (name,database),implementation in zip(helpers,(OgnROS2CameraHelper,OgnROS2CameraInfoHelper)):
            db=database(og.Controller.node(f'{graph}/{name}'))
            db.inputs._prefetch()
            if implementation.compute(db) is False:
                raise RuntimeError('Could not initialize '+name)
        for _ in range(30):
            await app.next_update_async()
        ensure_writer_gate(stage)
        result['gates']={str(p.GetPath()):str(p.GetAttribute('inputs:step').Get())
                         for p in stage.Traverse() if 'IsaacSimulationGate' in str(p.GetAttribute('node:type').Get())}
        if not result['gates']:
            raise RuntimeError('Camera writers did not attach: no simulation gates')
        stage.GetRootLayer().Save()
        result['ok']=True
    except Exception:
        result['ok']=False
        result['error']=traceback.format_exc()
    destination.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('[welding camera repair]',json.dumps(result))


async def watch():
    """Re-attach the session gate after Stop/Play or stage reload."""
    app=omni.kit.app.get_app()
    await repair()
    while True:
        for _ in range(30):
            await app.next_update_async()
        stage=omni.usd.get_context().get_stage()
        if stage:
            ensure_writer_gate(stage)


def start():
    return asyncio.ensure_future(watch())
