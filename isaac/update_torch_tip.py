"""Update only the working TCP in an existing scene and exported URDF, with backup."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
from pxr import Usd,UsdGeom
from attach_torch import origin
from torch_tip import working_tip

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',required=True,type=Path)
    parser.add_argument('--scene',required=True,type=Path)
    parser.add_argument('--package',type=Path,default=Path(__file__).resolve().parents[1]/'src/ur10_isaac_moveit_config')
    args=parser.parse_args()
    source=Usd.Stage.Open(str(args.source))
    matrix,extension=working_tip(source)
    stage=Usd.Stage.Open(str(args.scene))
    if not stage.GetPrimAtPath('/World/ur10/wrist_3_link/WeldingTorch'):
        parser.error('Scene has no mounted WeldingTorch')
    backup=args.scene.with_name(args.scene.name+'.bak_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    shutil.copy2(args.scene,backup)
    stage.SetEditTarget(stage.GetRootLayer())
    tcp=UsdGeom.Xform.Define(stage,'/World/ur10/wrist_3_link/WeldingTorch/WeldingTCP')
    tcp.ClearXformOpOrder();tcp.AddTransformOp().Set(matrix)
    stage.GetRootLayer().Save()
    xacro=args.package/'config/welding_torch.xacro'
    # Preserve the original xacro namespace prefix when serializing.
    ET.register_namespace('xacro','http://www.ros.org/wiki/xacro')
    tree=ET.parse(xacro)
    tree.getroot().find(".//joint[@name='torch_tcp_joint']/origin").attrib.update(origin(matrix))
    ET.indent(tree);tree.write(xacro,encoding='utf-8',xml_declaration=True)
    path=args.package/'config/torch_geometry.json'
    data=json.loads(path.read_text())
    data.update(tcp='WeldingTCP',tcp_definition='center of forward CopperContactTip face',
                contact_extension_from_nozzle_m=extension,tcp_origin=origin(matrix))
    path.write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps({'tcp':origin(matrix),'extension_from_nozzle_m':extension,'backup':str(backup)},indent=2))

if __name__=='__main__':main()
