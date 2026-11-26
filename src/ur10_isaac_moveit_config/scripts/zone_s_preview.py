"""Preview reconstruction from the completed Windows scanner's stereo pairs."""

import json
from pathlib import Path
import re

import numpy as np


class ZoneSPreview:
    def __init__(self, config_path):
        config = json.loads(Path(config_path).read_text())
        self.captures_path = Path(config['captures_path'])
        self.frame_signature = config['frame_signature']
        self.model = {key: np.asarray(value) for key, value in config['stereo'].items()}
        self.world_rotation = np.asarray(config['world_rotation'])
        self.world_translation = np.asarray(config['world_translation_m'])

    def reconstruct(self, capture, frame_signature):
        if frame_signature != self.frame_signature:
            raise ValueError('Scanner calibration changed; update the zone S preview reference')
        # The archived stereo correspondences come from structured-light matches.
        if not re.fullmatch(r'[0-9a-f]{32}', capture):
            raise ValueError('Invalid scanner capture identifier')
        path = self.captures_path / f'dense-job-{capture}' / 'measured-cloud.npz'
        with np.load(path) as archive:
            pixels = [archive['pixel0'], archive['pixel1']]
        rays = []
        for i, pair in enumerate(pixels):
            xy = (pair-self.model['principal_px'][i])/self.model['focal_px'][i]
            rays.append(np.column_stack((xy, np.ones(len(xy)))))
        r = self.model['right_from_left_rotation']
        t = self.model['right_from_left_translation_mm']
        a, b = rays
        rotated = a @ r.T
        line_right = np.cross(t, rotated)
        line_left = np.cross(t, b) @ r
        residual_px = np.abs(np.einsum('ij,ij->i', b, line_right))
        residual_px *= self.model['focal_px'].mean()
        residual_px /= np.linalg.norm(np.column_stack((line_right[:,:2],line_left[:,:2])),axis=1)
        keep = np.isfinite(residual_px) & (residual_px <= 2.5)
        if np.count_nonzero(keep) < 1000 or keep.mean() < .40:
            raise ValueError('Zone S model does not agree with the new stereo pairs')
        # Find the closest points on the two rays in the left camera frame.
        u, v = a[keep], b[keep] @ r
        origin_right = -r.T @ t
        uu, vv, uv = (np.einsum('ij,ij->i', x, y) for x,y in ((u,u),(v,v),(u,v)))
        uo, vo = u @ origin_right, v @ origin_right
        cross = uu*vv-uv*uv
        l_depth = (vv*uo-uv*vo)/cross
        r_depth = (uv*uo-uu*vo)/cross
        valid = (l_depth > 180) & (l_depth < 380) & (r_depth > 180) & (r_depth < 380)
        triangulated = (u*l_depth[:,None]+origin_right+v*r_depth[:,None])/2
        native = (triangulated[valid] @ self.model['native_from_left_rotation'].T + self.model['left_center_mm'])/1000
        if len(native) < 1000:
            raise ValueError('Too few valid points in the scanner working range')
        return native @ self.world_rotation.T + self.world_translation
