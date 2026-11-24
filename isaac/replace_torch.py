"""Replace torch geometry while retaining the calibrated attachment and TCP."""
import argparse
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from attach_torch import mesh_triangles, origin, stl
from configure_torch_collision import envelopes
from torch_tip import working_tip


CONTACT_TIP_EXTENSION_M = .003


def update_scene(scene, output, source, boxes):
    def asset_path(file):
        try:
            return Path(os.path.relpath(file.resolve(), output.parent.resolve())).as_posix()
        except ValueError:
            return file.resolve().as_posix()

    path = '/World/ur10/wrist_3_link/WeldingTorch'
    original = Usd.Stage.Open(str(scene))
    if not original.GetPrimAtPath(path+'/WeldingTCP'):
        raise ValueError('Expected an existing mounted torch with a calibrated WeldingTCP')
    if output.exists():
        raise ValueError('Choose a new output USD layer')
    stage = Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths = [asset_path(scene)]
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    stage.OverridePrim(path).GetReferences().SetReferences([Sdf.Reference(asset_path(source))])
    for name in ('HoseEnvelope', 'CollisionEnvelope', 'ReplacementCollision'):
        if stage.GetPrimAtPath(path+'/'+name):
            stage.OverridePrim(path+'/'+name).SetActive(False)
    for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
    # A new layer can also be applied over a previously generated overlay.
    group = 'TorchCollision'
    while stage.GetPrimAtPath(path+'/'+group):
        stage.OverridePrim(path+'/'+group).SetActive(False)
        group += '_New'
    for index, (_, low, high) in enumerate(boxes):
        cube = UsdGeom.Cube.Define(stage, f'{path}/{group}/Part_{index}')
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*((low+high)/2)))
        cube.AddScaleOp().Set(Gf.Vec3d(*(high-low)))
        cube.CreateVisibilityAttr('invisible')
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
    stage.GetRootLayer().Save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--package', type=Path,
                        default=Path(__file__).resolve().parents[1]/'src/ur10_isaac_moveit_config')
    parser.add_argument('--scene', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if bool(args.scene) != bool(args.output):
        parser.error('Supply both --scene and --output to update an Isaac scene')
    source = Usd.Stage.Open(str(args.source))
    mount = source.GetPrimAtPath(str(source.GetDefaultPrim().GetPath())+'/MountFrame')
    if not mount or UsdGeom.GetStageMetersPerUnit(source) != 1.:
        parser.error('Expected MountFrame and a USD asset in meters')
    mount_inverse = UsdGeom.XformCache().GetLocalToWorldTransform(mount).GetInverse()
    visuals, collisions, _ = mesh_triangles(source, mount_inverse)
    boxes, count = envelopes(source)
    tip, extension = working_tip(source)
    displacement = CONTACT_TIP_EXTENSION_M * np.array(tip.TransformDir(Gf.Vec3d(0, 0, 1)))
    contact_tip = str(source.GetDefaultPrim().GetPath()) + '/LowerPivot/Torch/CopperContactTip'
    for index, (name, low, high) in enumerate(boxes):
        if name == contact_tip:
            boxes[index] = (name, low + np.minimum(displacement, 0), high + np.maximum(displacement, 0))
            break
    else:
        raise ValueError('Collision envelope for CopperContactTip is missing')
    config = args.package/'config'
    path = config/'welding_torch.xacro'
    namespace = 'http://www.ros.org/wiki/xacro'
    ET.register_namespace('xacro', namespace)
    tree = ET.parse(path)
    macro = tree.find(f'{{{namespace}}}macro')
    link = macro.find("link[@name='welding_torch_link']")
    tcp = macro.find("joint[@name='torch_tcp_joint']/origin")
    macro.set('params', 'parent mount_yaw:=0')
    for prop in macro.findall(f'{{{namespace}}}property'):
        if prop.get('name') in ('nozzle_trim_y', 'nozzle_trim_z'):
            macro.remove(prop)
    for tag in ('visual', 'collision'):
        for item in link.findall(tag):
            link.remove(item)
    asset = args.package/'assets'/args.source.name
    if args.source.resolve() != asset.resolve():
        shutil.copyfile(args.source, asset)
    if args.scene:
        update_scene(args.scene, args.output, asset, boxes)
    output = args.package/'meshes/torch'
    stl(output/'collision.stl', collisions)
    for index, (color, triangles) in enumerate(sorted(visuals.items())):
        filename = f'visual_{index}.stl'
        stl(output/filename, triangles)
        visual = ET.SubElement(link, 'visual')
        ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh', {
            'filename': 'package://ur10_isaac_moveit_config/meshes/torch/'+filename})
        material = ET.SubElement(visual, 'material', {'name': f'torch_color_{index}'})
        ET.SubElement(material, 'color', {'rgba': ' '.join(map(str, (*color, 1.)))})
    for index, (_, low, high) in enumerate(boxes):
        collision = ET.SubElement(link, 'collision', {'name': f'cad_part_{index}'})
        ET.SubElement(collision, 'origin', {'xyz': ' '.join(map(str, (low+high)/2)), 'rpy': '0 0 0'})
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', {'size': ' '.join(map(str, high-low))})
    ET.indent(tree)
    path.write_bytes(ET.tostring(tree.getroot(), encoding='utf-8', xml_declaration=True)+b'\n')
    report = dict(source=asset.name, mount='MountFrame', tcp='WeldingTCP',
                  tcp_definition='preserved calibrated TCP', tcp_origin=dict(tcp.attrib),
                  asset_contact_tip_origin=origin(tip), contact_tip_from_nozzle_frame_m=extension,
                  collision_triangles=len(collisions), visual_triangles=sum(map(len, visuals.values())),
                  visual_materials=len(visuals))
    (config/'torch_geometry.json').write_text(json.dumps(report, indent=2)+'\n', newline='\n')
    collision_data = dict(source=asset.name, method='conservative per-component CAD bounding boxes',
                          box_count=len(boxes), visible_components=count, padding_m=.0005,
                          contact_tip_extension_m=CONTACT_TIP_EXTENSION_M,
                          boxes=[dict(source=name, center=((lo+hi)/2).tolist(), size=(hi-lo).tolist())
                                 for name,lo,hi in boxes])
    (config/'torch_collision.json').write_text(json.dumps(collision_data, indent=2)+'\n', newline='\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
