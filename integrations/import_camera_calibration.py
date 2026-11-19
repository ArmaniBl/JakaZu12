"""Validate a portable calibration bundle and prepare a native Windows setup.

Only camera ID spelling and local evidence paths change. No robot transform
or metric-scale verification is invented. Stop the scanner before --install.
"""
import argparse
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import time
import numpy as np

parser=argparse.ArgumentParser()
parser.add_argument('--bundle',type=Path,required=True)
parser.add_argument('--scanner-root',type=Path,required=True)
parser.add_argument('--install',action='store_true')
args=parser.parse_args()
bundle=args.bundle.resolve();root=args.scanner_root.resolve()
for line in (bundle/'SHA256SUMS.txt').read_text(encoding='utf-8').splitlines():
    expected,name=line.split(None,1);path=(bundle/name.strip()).resolve()
    if not path.is_relative_to(bundle) or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
        raise ValueError('Calibration checksum mismatch: '+name)
source=json.loads((bundle/'candidate.json').read_text(encoding='utf-8'))
session=json.loads((bundle/'session.json').read_text(encoding='utf-8'))
source_hash=hashlib.sha256((bundle/'candidate.json').read_bytes()).hexdigest()
if not session.get('ready') or session.get('accepted_sha256')!=source_hash or session.get('candidate')!=source:
    raise ValueError('Source calibration was not consistently accepted')
sys.path.insert(0,str(root/'сканер/scripts'))
from setup_fit import fit
def report(p):
    return json.loads((bundle/p['directory']/'target-detection.json').read_text(encoding='utf-8'))
reproduced=fit([report(p) for p in session['poses']],report(session['validation']))
print(json.dumps({'source_rms_px':source['stereo_rms_px'],'replay_rms_px':reproduced['stereo_rms_px'],
    'replay_control_px':reproduced['independent_rms_px'],
    'intrinsics_difference':[ (np.array(a['K'])-np.array(b['K'])).tolist() for a,b in zip(source['cameras'],reproduced['cameras'])],
    'translation_difference':(np.array(source['stereo_T_grid_units'])-np.array(reproduced['stereo_T_grid_units'])).tolist()},indent=2))
if not reproduced['quality_passed'] or abs(reproduced['stereo_rms_px']-source['stereo_rms_px'])>.05:
    raise ValueError('Independent replay of recorded observations failed')
# Use the result computed by the current Windows/OpenCV implementation from
# the supplied measurements, rather than relabel the old numerical solution.
# Its independent held-out pair must pass the normal fit quality criteria.
for old,new in zip(source['cameras'],reproduced['cameras']):
    a,b=np.asarray(old['K']),np.asarray(new['K'])
    if max(abs(b[0,0]/a[0,0]-1),abs(b[1,1]/a[1,1]-1))>.02:
        raise ValueError('Replay focal lengths diverge from the supplied result')
config=(root/'config.cmd').read_text(encoding='utf-8-sig')
import re
serials=[re.search(r'SPECTRUM_CAMERA_'+str(i)+r'_SERIAL=([^"\r\n]+)',config).group(1) for i in (0,1)]
candidate=deepcopy(reproduced)
mapping={}
for index,serial in enumerate(serials):
    legacy=source['cameras'][index]['device_id']
    if legacy!=serial and not legacy.endswith('-'+serial):
        raise ValueError('Configured camera order differs from measured calibration')
    mapping[legacy]=serial;candidate['cameras'][index]['device_id']=serial
target=root/'runtime/scanner/spectrum/setup'
audit=dict(source_candidate_sha256=source_hash,source_bundle=str(bundle),
           camera_id_mapping=mapping,changed_parameters=['recomputed from recorded measurements on Windows','device_id spelling'],
           replay_stereo_rms_px=reproduced['stereo_rms_px'],
           replay_control_rms_px=reproduced['independent_rms_px'],
           metric_scale_verified=False,robot_registration_present=False,axis_activated=False)
if args.install:
    if target.exists():
        shutil.copytree(target,target.with_name('setup-backup-'+time.strftime('%Y%m%d-%H%M%S')))
    target.mkdir(parents=True,exist_ok=True)
    shutil.copytree(bundle,target/'imported-original',dirs_exist_ok=True)
    shutil.copytree(bundle/'poses',target/'poses',dirs_exist_ok=True)
    for entry in [*session['poses'],session.get('validation'),session.get('latest')]:
        if entry:
            entry['directory']=str(target/'poses'/Path(entry['directory']).name)
    content=(json.dumps(candidate,indent=2,ensure_ascii=False)+'\n').encode('utf-8')
    (target/'candidate.json').write_bytes(content)
    session.update(candidate=candidate,accepted_sha256=hashlib.sha256(content).hexdigest(),
                   axis=None,operation='IDLE',error=None)
    # Existing current axis, if any, must not become active after this import.
    axis=target/'axis.json'
    if axis.exists():axis.rename(target/('axis-disabled-'+time.strftime('%Y%m%d-%H%M%S')+'.json'))
    (target/'session.json').write_text(json.dumps(session,ensure_ascii=False,indent=2),encoding='utf-8')
    (target/'calibration-import.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(audit,ensure_ascii=False,indent=2))
