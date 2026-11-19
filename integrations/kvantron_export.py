"""Read-only scan/CAD contract. Installed beside the source scanner service."""
import hashlib
import json
import time
import numpy as np


def export_bundle(context, part, camera_frame, camera_T_display, reusable, task_id, capture_id):
    cloud = context
    if not reusable or not camera_frame or not capture_id:
        raise ValueError('Нужен новый измеренный скан в стабильной калиброванной системе камер')
    if cloud.get('synthetic') is True or not cloud['provenance'].get('calibration_sha256',
                                            cloud['provenance'].get('reconstruction', {}).get('calibration_sha256')):
        raise ValueError('Нет измерительной калибровки скана')
    from adaptive_weld.geometry import transform_matrix
    scale = float(cloud['pose']['scale'])
    if not np.isfinite(scale) or scale <= 0 or (cloud['unit'] == 'mm' and not np.isclose(scale, 1)):
        raise ValueError('Некорректный метрический масштаб')
    camera_T_display = np.asarray(camera_T_display, float)
    points = np.asarray(cloud['points'], float)*scale
    camera_points = points@camera_T_display[:3, :3].T+camera_T_display[:3, 3]
    display_T_cad = np.linalg.inv(transform_matrix(cloud['pose']['position'], cloud['pose']['rotation']))@transform_matrix(part.position, part.rotation)
    bundle = dict(schema='kvantron-moveit/v1', units='mm', created_unix=time.time(),
                  frame_signature=camera_frame, task_id=task_id, capture_id=capture_id,
                  part_name=part.name, camera_points_mm=camera_points.tolist(),
                  camera_T_cad_mm=(camera_T_display@display_T_cad).tolist(),
                  vertices_mm=np.asarray(part.mesh.vertices).tolist(),
                  triangles=np.asarray(part.mesh.faces).tolist(),
                  calibration_sha256=cloud['provenance'].get('calibration_sha256',
                      cloud['provenance'].get('reconstruction', {}).get('calibration_sha256')))
    bundle['identity'] = hashlib.sha256(json.dumps({k:v for k,v in bundle.items() if k!='created_unix'},
                                                  sort_keys=True, allow_nan=False).encode()).hexdigest()
    return bundle
