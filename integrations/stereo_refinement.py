"""Bounded, regularized angular refinement for an unregistered cloud preview.

Intrinsics, camera-0 coordinates, stereo translation and metric scale stay fixed.
This is a per-scan estimate from measured projector correspondences, not an
accepted calibration. A separate subset validates the estimate before use.
"""
import cv2
import numpy as np
from scipy.optimize import least_squares


def triangulate(pixels, cameras, rotation, translation):
    normalized = [cv2.undistortPoints(np.asarray(p, dtype=float)[:, None, :],
        np.asarray(c['K']), np.asarray(c['distortion'])).reshape(-1, 2).T
        for p, c in zip(pixels, cameras)]
    translation = np.asarray(translation, dtype=float).reshape(3, 1)
    h = cv2.triangulatePoints(np.c_[np.eye(3), np.zeros(3)],
                              np.c_[rotation, translation], *normalized)
    finite = np.abs(h[3]) > 1e-10
    xyz = (h[:3]/np.where(finite, h[3], 1)).T
    other = xyz@rotation.T+translation.ravel()
    positive = finite & np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0) & (other[:, 2] > 0)
    residuals = []
    for p, c, rv, tv in zip(pixels, cameras,
            [np.zeros(3), cv2.Rodrigues(rotation)[0]], [np.zeros(3), translation]):
        projected, _ = cv2.projectPoints(xyz, rv, tv, np.asarray(c['K']), np.asarray(c['distortion']))
        residuals.append(np.linalg.norm(projected[:, 0]-p, axis=1))
    return xyz, positive, np.maximum(*residuals)


def refine_preview_rotation(pixels, cameras, rotation, translation):
    """Reject ambiguous/large corrections; never relax the 2 px geometry gate."""
    pixels = [np.asarray(p, dtype=float) for p in pixels]
    if (len(pixels) != 2 or pixels[0].shape != pixels[1].shape
            or pixels[0].ndim != 2 or pixels[0].shape[1] != 2
            or len(pixels[0]) < 1000 or not all(np.isfinite(p).all() for p in pixels)):
        raise ValueError('Preview refinement needs at least 1000 measured stereo pairs')
    if any(np.ptp(p, axis=0).min() < 30 for p in pixels):
        raise ValueError('Measured pairs cover too narrow an image region')
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float).reshape(3)
    _, _, before = triangulate(pixels, cameras, rotation, translation)
    if not .5 < np.median(before) < 6 or np.percentile(before, 95) > 10:
        raise ValueError('Stereo mismatch is outside bounded preview refinement')
    normalized = []
    for p, c in zip(pixels, cameras):
        uv = cv2.undistortPoints(p[:, None, :], np.asarray(c['K']),
                                np.asarray(c['distortion'])).reshape(-1, 2)
        normalized.append(np.c_[uv, np.ones(len(uv))])
    x, y, z = translation
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    focal = np.mean([np.diag(np.asarray(c['K']))[:2].mean() for c in cameras])
    # Keep validation observations out of the optimization. Uniform subsampling
    # also bounds work and preserves coverage over the sorted projector keys.
    train = np.arange(0, len(pixels[0]), max(2, int(np.ceil(len(pixels[0])/6000))))
    a, b = [v[train] for v in normalized]
    def residual(delta):
        essential = cross@(cv2.Rodrigues(delta)[0]@rotation)
        ea, eb = a@essential.T, b@essential
        denominator = np.sqrt(np.sum(ea[:, :2]**2+eb[:, :2]**2, axis=1))
        epipolar = np.sum(b*ea, axis=1)/np.maximum(denominator, 1e-12)*focal
        # A strong prior avoids a planar scene moving the inferred baseline or
        # choosing a large angular solution with an artificially perfect fit.
        return np.r_[epipolar/np.sqrt(len(a)), delta*300.]
    fit = least_squares(residual, np.zeros(3), max_nfev=60)
    angle = float(np.degrees(np.linalg.norm(fit.x)))
    if not fit.success or not np.isfinite(fit.x).all() or angle > .2:
        raise ValueError('Angular correction exceeds 0.2 degrees')
    refined = cv2.Rodrigues(fit.x)[0]@rotation
    _, positive, after = triangulate(pixels, cameras, refined, translation)
    validation = np.ones(len(after), dtype=bool); validation[train] = False
    valid_errors = after[validation]
    median, p95 = float(np.median(valid_errors)), float(np.percentile(valid_errors, 95))
    if (median > 1 or p95 > 1.5 or np.mean(positive[validation] & (valid_errors <= 2)) < .97
            or median > np.median(before[validation])*.5):
        raise ValueError(f'Angular estimate failed held-out stereo geometry checks: median={median:.3f}px, p95={p95:.3f}px')
    return refined, dict(method='regularized_rotation_only', preliminary=True,
        accepted_calibration=False, execution_ready=False, rotation_delta_deg=np.degrees(fit.x).tolist(),
        angle_deg=angle, training_pairs=len(train), validation_pairs=int(validation.sum()),
        before_median_px=float(np.median(before)), validation_median_px=median,
        validation_p95_px=p95, translation_changed=False, intrinsics_changed=False,
        filled_or_cad_generated_points=0)
