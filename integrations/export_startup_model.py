"""Export the selected Windows scanner STEP as a portable startup mesh."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

parser=argparse.ArgumentParser()
parser.add_argument('scanner_root',type=Path)
parser.add_argument('output',type=Path)
args=parser.parse_args()
root=args.scanner_root.resolve()
sys.path.insert(0,str(root))
from adaptive_weld.geometry import import_parts

runtime=root/'runtime/scanner'
project=json.loads((runtime/'project.json').read_text(encoding='utf-8'))
parts=[p for p in project['parts'] if p.get('enabled') and p.get('role')=='workpiece']
tasks=json.loads((runtime/'spectrum/tasks.json').read_text(encoding='utf-8'))
active=next((t for t in tasks['tasks'] if t['id']==tasks['active_id']),None)
if active:
    parts=[p for p in parts if p['id']==active['part_id']]
if len(parts)!=1:raise SystemExit('Select exactly one active scanner workpiece')
part=parts[0]
path=runtime/'saved-parts'/part['file']
geometry=import_parts(path)
if len(geometry)!=1:raise SystemExit('Expected a single-body workpiece')
mesh=geometry[0].mesh
data=dict(schema='jaka-startup-workpiece/v1',units='mm',part_name=part['name'],
          source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
          vertices_mm=mesh.vertices.tolist(),triangles=mesh.faces.tolist(),
          initial_pose=dict(position_m=[-.38,-.69,0.],quaternion_xyzw=[0.,0.,-.7071067811865475,.7071067811865476]),
          placement_verified=False)
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(data,indent=2),encoding='utf-8')
print(json.dumps(dict(part_name=part['name'],vertices=len(mesh.vertices),triangles=len(mesh.faces),output=str(args.output)),ensure_ascii=False))
