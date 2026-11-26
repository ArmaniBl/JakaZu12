"""Decode the completed Windows capture without the legacy stereo calibration."""
import json
from pathlib import Path
import sys

import cv2
import numpy as np


def main():
    capture, output = map(Path, sys.argv[1:])
    scripts = capture.parents[1] / 'scripts'
    sys.path.insert(0, str(scripts))
    from dense_reconstruct import centroids, decode
    from image_io import read_image

    manifest = json.loads((capture / 'manifest.json').read_text())
    if manifest['status'] != 'complete':
        raise ValueError('Structured-light capture is incomplete')
    final = next(p['index'] for p in manifest['patterns'] if p['kind'] == 'white_check')
    pairs = []
    for camera in range(2):
        first = read_image(str(capture / f'camera-{camera}-000.png'), cv2.IMREAD_GRAYSCALE)
        last = read_image(str(capture / f'camera-{camera}-{final:03d}.png'), cv2.IMREAD_GRAYSCALE)
        if first is None or last is None:
            raise ValueError('Capture is missing motion-check frames')
        shift, response = cv2.phaseCorrelate(first.astype('float32'), last.astype('float32'))
        if not np.isfinite(shift).all() or not np.isfinite(response) or response < .2 or np.linalg.norm(shift) > 1.5:
            raise ValueError('Scene changed during structured-light capture')
        x, y, valid, _ = decode(capture, camera, manifest)
        pairs.append(centroids(x, y, valid, manifest['display']['pixels_w']))
    _, left, right = np.intersect1d(pairs[0][0], pairs[1][0], return_indices=True)
    temporary = output.with_suffix('.tmp.npz')
    np.savez_compressed(temporary, pixel0=pairs[0][1][left], pixel1=pairs[1][1][right])
    temporary.replace(output)


if __name__ == '__main__':
    main()
