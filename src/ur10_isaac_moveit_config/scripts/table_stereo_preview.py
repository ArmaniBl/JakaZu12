"""Visual-only stereo reconstruction using surveyed tabletop references."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

import numpy as np


class TableStereoPreview:
    def __init__(self, config_path):
        config_path = Path(config_path)
        config = json.loads(config_path.read_text())
        self.reference = config_path.parent / config['reference_cloud']
        self.archives = Path(config['captures_path'])
        self.devices = config['camera_ids']
        self.tabletop_z = float(config['tabletop_z_m'])
        model_path = config_path.parent / config['camera_model']
        model = model_path.read_bytes()
        self.cameras = json.loads(model)['models']
        self.signature = 'table-preview:' + hashlib.sha256(model).hexdigest()
        self.integrations = config_path.parent.parent / 'integrations'

    def reference_cloud(self):
        return np.load(self.reference, allow_pickle=False)

    def reconstruct(self, capture_id):
        if not re.fullmatch(r'[0-9a-f]{32}', capture_id):
            raise ValueError('Invalid table preview capture identifier')
        folder = self.archives / f'dense-job-{capture_id}'
        manifest = json.loads((folder / 'manifest.json').read_text())
        if manifest['status'] != 'complete' or manifest.get('device_ids') != self.devices:
            raise ValueError('Capture uses different cameras or is incomplete')
        archive_path = folder / 'table-stereo-pairs.npz'
        if not archive_path.is_file():
            def windows_path(path):
                return subprocess.check_output(['wslpath', '-w', str(path)], text=True).strip()

            command = [
                'powershell.exe', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
                '-File', windows_path(self.integrations / 'decode_table_pairs.ps1'),
                '-Script', windows_path(self.integrations / 'decode_table_pairs.py'),
                '-Capture', windows_path(folder), '-Output', windows_path(archive_path),
            ]
            result = subprocess.run(command, capture_output=True, timeout=55)
            if result.returncode:
                detail = result.stderr.decode('utf-8', errors='replace').strip()
                raise ValueError('Structured-light decoding failed: ' + detail[-1000:])
        with np.load(archive_path, allow_pickle=False) as archive:
            pairs = [archive[f'pixel{i}'] for i in range(2)]
        return self.triangulate(pairs)

    def triangulate(self, pairs):
        from scipy.spatial.transform import Rotation

        rays, poses, observed = [], [], []
        for pixels, camera in zip(pairs, self.cameras):
            pixels = np.asarray(pixels) / 2.
            k = np.asarray(camera['K'])
            distorted = (pixels-k[:2, 2])/np.diag(k)[:2]
            radial = float(camera['distortion'][0])
            xy = distorted.copy()
            for _ in range(6):
                xy = distorted / (1. + radial*np.sum(xy**2, axis=1, keepdims=True))
            rays.append(np.column_stack((xy,np.ones(len(xy)))))
            poses.append((Rotation.from_rotvec(camera['rvec']).as_matrix(), np.asarray(camera['tvec_mm'])))
            observed.append((pixels,k,radial))

        directions = [ray @ pose[0] for ray,pose in zip(rays,poses)]
        origins = [-r.T @ t for r,t in poses]
        u,v = directions
        delta = origins[1]-origins[0]
        uu = np.sum(u*u,axis=1)
        vv = np.sum(v*v,axis=1)
        uv = np.sum(u*v,axis=1)
        determinant = uu*vv-uv**2
        separation = determinant>1e-10
        denominator = np.where(separation,determinant,1.)
        ud, vd = u @ delta, v @ delta
        a = (vv*ud-uv*vd)/denominator
        b = (uv*ud-uu*vd)/denominator
        points = (origins[0]+a[:,None]*u+origins[1]+b[:,None]*v)/2.
        valid = separation & np.isfinite(points).all(axis=1) & (a>0) & (b>0)
        for (r,t),(pixels,k,radial) in zip(poses,observed):
            camera = points@r.T+t
            positive = camera[:,2]>0
            xy = camera[:,:2] / np.where(positive,camera[:,2],1.)[:,None]
            xy *= 1. + radial*np.sum(xy**2,axis=1,keepdims=True)
            projected = xy*np.diag(k)[:2]+k[:2,2]
            valid &= positive & (np.linalg.norm(projected-pixels,axis=1)<=3.)
        if np.count_nonzero(valid)<40:
            raise ValueError('Too few stereo pairs match the tabletop preview model')
        return points[valid]/1000 + [0.,0.,self.tabletop_z]
