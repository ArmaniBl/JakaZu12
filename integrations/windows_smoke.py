"""Smoke-test the Windows web/CAD stack in a temporary runtime; no hardware."""
import argparse
import json
import os
from pathlib import Path
import sys
import tempfile

parser=argparse.ArgumentParser()
parser.add_argument('--scanner-root',required=True)
args=parser.parse_args()
root=Path(args.scanner_root).resolve()
sys.path.insert(0,str(root))
with tempfile.TemporaryDirectory(prefix='welding-smoke-') as runtime:
    os.environ.update(SCANNER_ROOT=str(root),SCANNER_RUNTIME=runtime,
                      SCANNER_INITIAL_SCENE='empty',KVANTRON_MOVEIT_BRIDGE='1')
    from scanner_app.app import app
    client=app.test_client()
    checks={}
    for path in ('/api/health','/','/api/spectrum/state'):
        response=client.get(path)
        assert response.status_code==200,(path,response.status_code,response.get_data(as_text=True)[:500])
        checks[path]=response.status_code
    response=client.get('/api/spectrum/moveit-export')
    assert response.status_code==400,(response.status_code,response.get_data(as_text=True))
    checks['unmeasured_export_rejected']=True
    for path in ('/api/jaka/connect','/api/workflow/start'):
        response=client.post(path,json={})
        assert response.status_code==409,(path,response.status_code)
    checks['direct_robot_control_blocked']=True
    print(json.dumps(checks,ensure_ascii=False,indent=2))
