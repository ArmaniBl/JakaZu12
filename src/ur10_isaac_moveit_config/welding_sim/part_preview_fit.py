"""Automatic CAD registration for visual previews. No robot calibration or motion."""
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def mesh_arrays(vertices_mm, triangles):
    vertices=np.asarray(vertices_mm,dtype=float)*.001
    faces=np.asarray(triangles)
    if (vertices.ndim!=2 or vertices.shape[1]!=3 or not 4<=len(vertices)<=100000
            or faces.ndim!=2 or faces.shape[1]!=3 or not 4<=len(faces)<=20000
            or faces.dtype.kind not in 'iu' or faces.min()<0 or faces.max()>=len(vertices)
            or not np.isfinite(vertices).all()):
        raise ValueError('Invalid workpiece mesh')
    return vertices, faces


def top_surface(vertices_mm,triangles,spacing=.0015):
    vertices, faces = mesh_arrays(vertices_mm, triangles)
    extent=np.ptp(vertices,axis=0)
    if extent[2]>.3*min(extent[:2]) or min(extent)<.0001:
        raise ValueError('Automatic preview currently requires a thin part with local Z thickness')
    zmax=vertices[:,2].max()
    points=[]
    for ids in faces:
        a,b,c=vertices[ids]
        if max(abs(np.array([a[2],b[2],c[2]])-zmax))>1e-6:continue
        count=min(200,max(2,int(max(np.linalg.norm(b-a),np.linalg.norm(c-a))/spacing)+1))
        u,v=np.meshgrid(np.linspace(0,1,count),np.linspace(0,1,count));mask=u+v<=1
        points.append(a+(b-a)*u[mask,None]+(c-a)*v[mask,None])
    if not points:raise ValueError('CAD has no horizontal top surface')
    points=np.concatenate(points)
    _,indices=np.unique(np.floor(points/spacing),axis=0,return_index=True)
    return vertices,points[indices]


def dominant_plane(points):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<100 or not np.isfinite(points).all():
        raise ValueError('At least 100 finite scan points required')
    sample=points[::max(1,len(points)//12000)]
    rng=np.random.default_rng(31);best=np.zeros(len(sample),bool)
    for _ in range(160):
        a,b,c=sample[rng.choice(len(sample),3,False)];n=np.cross(b-a,c-a);length=np.linalg.norm(n)
        if length<1e-7:continue
        n/=length
        if abs(n[2])<math.cos(math.radians(25)):continue
        mask=np.abs((sample-a)@n)<.0018
        if mask.sum()>best.sum():best=mask
    if best.sum()<100 or best.mean()<.35:raise ValueError('No dominant near-horizontal workpiece surface')
    selected=sample[best]
    # Nearby fixtures can be coplanar with the part. Keep a connected surface,
    # rather than fitting a rectangle around the entire illuminated scene.
    _,indices=np.unique(np.floor(selected/.001),axis=0,return_index=True)
    selected=selected[indices]
    pairs=cKDTree(selected).query_pairs(.004,output_type='ndarray')
    edges=np.vstack([pairs,pairs[:,::-1]])
    graph=coo_matrix((np.ones(len(edges)),(edges[:,0],edges[:,1])),shape=(len(selected),len(selected))).tocsr()
    _,labels=connected_components(graph,directed=False)
    selected=selected[labels==np.argmax(np.bincount(labels))]
    if len(selected)<100:raise ValueError('No connected workpiece surface')
    origin=selected.mean(0)
    _,s,v=np.linalg.svd(selected-origin,full_matrices=False)
    if s[1]/math.sqrt(len(selected))<.005:raise ValueError('Measured surface is too narrow')
    normal=v[-1]
    if normal[2]<0:normal=-normal
    x=v[0];y=np.cross(normal,x)
    basis=np.column_stack([x,y,normal])
    return selected,origin,basis


def rigid2(source,target,weights):
    weights=weights/weights.sum();a=weights@source;b=weights@target
    u,_,vt=np.linalg.svd((source-a).T@((target-b)*weights[:,None]))
    rotation=vt.T@u.T
    if np.linalg.det(rotation)<0:vt[-1]*=-1;rotation=vt.T@u.T
    return rotation,b-rotation@a


def fit_preview(points_world,vertices_mm,triangles, reference=None, *, table_top=None, allow_side_view=False):
    vertices, faces = mesh_arrays(vertices_mm, triangles)
    extent = np.ptp(vertices, axis=0)
    if extent[2] > .3 * min(extent[:2]) and table_top is not None:
        return fit_supported_preview(points_world, vertices, faces, reference, table_top, allow_side_view)
    vertices,model=top_surface(vertices_mm,triangles)
    scan,origin,basis=dominant_plane(points_world)
    if table_top is not None:
        # A flat part rests on the table. Use the scan for XY and yaw and
        # compare its observed surface to the height of the supported CAD.
        top = table_top + extent[2]
        support_error = float(np.percentile(np.abs(scan[:,2]-top),95))
        if support_error > .008:
            raise ValueError('Measured top surface height disagrees with the CAD on the table')
        basis = np.eye(3)
    target=(scan-origin)@basis[:,:2]
    # Uniform samples prevent a densely reconstructed corner dominating the fit.
    _,indices=np.unique(np.floor(target/.001),axis=0,return_index=True);target=target[indices]
    source=model[:,:2]
    observed=np.ptp(np.percentile(target,[2,98],axis=0),axis=0)
    expected=np.sort(np.ptp(vertices[:,:2],axis=0))[::-1]
    if np.any(np.sort(observed)[::-1]/expected<.42) or np.any(np.sort(observed)[::-1]/expected>1.45):
        raise ValueError('Dominant scan surface dimensions do not match the selected CAD')
    tree=cKDTree(target);center=(np.percentile(target,2,axis=0)+np.percentile(target,98,axis=0))/2
    base=0. if np.ptp(source[:,0])>np.ptp(source[:,1]) else -math.pi/2
    accepted=[]
    partial=[]
    angles = [base + turn + delta for turn in (0., math.pi) for delta in (0., -.08, .08)]
    seeds = [(anchor, angle) for anchor in ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1))
             for angle in angles]
    if reference is not None:
        relative = basis.T @ reference[:3,:3]
        seeds.append((None, math.atan2(relative[1,0],relative[0,0])))
    for anchor, angle in seeds:
        if anchor != (0, 0) and accepted:
            break
        c,s=math.cos(angle),math.sin(angle);rotation=np.array([[c,-s],[s,c]])
        moved=source@rotation.T
        translation=center-(moved.min(0)+moved.max(0))/2
        if anchor is None:
            translation = (reference[:3,3] + reference[:3,2]*model[0,2] - origin) @ basis[:,:2]
        elif anchor != (0, 0):
            # A lateral view may omit one edge. Seed the match from either
            # measured edge as well as from the center of the visible region.
            translation += np.asarray(anchor)*np.maximum(np.ptp(moved,axis=0)-observed,0.)/2
        for iteration in range(70):
            moved=source@rotation.T+translation
            d1,j1=tree.query(moved);d2,j2=cKDTree(moved).query(target)
            keep1=d1<.008;keep2=d2<.008
            if keep1.mean()<.45 or keep2.mean()<.65:break
            if anchor != (0, 0) and np.prod(observed) < .75*np.prod(expected):
                visible = np.argpartition(d1, len(d1)//2)[:len(d1)//2]
                keep1[:] = False
                keep1[visible] = d1[visible] < .008
            a=np.concatenate([moved[keep1],moved[j2[keep2]]]);b=np.concatenate([target[j1[keep1]],target[keep2]])
            weights=np.r_[np.full(keep1.sum(),.5/keep1.sum()),np.full(keep2.sum(),.5/keep2.sum())]
            dr,dt=rigid2(a,b,weights);rotation=dr@rotation;translation=dr@translation+dt
            if np.linalg.norm(dt)<1e-6 and abs(dr[1,0])<1e-5:break
        moved=source@rotation.T+translation
        d1,_=tree.query(moved);d2,_=cKDTree(moved).query(target)
        rmse=float(np.sqrt((np.mean(d1**2)+np.mean(d2**2))/2))
        quality=dict(rmse_m=rmse,p95_m=float(max(np.percentile(d1,95),np.percentile(d2,95))),
                     model_coverage=float(np.mean(d1<.004)),scan_coverage=float(np.mean(d2<.004)),
                     point_count=len(scan),iterations=iteration+1)
        full_fit=rmse<=.004 and quality['p95_m']<=.008 and min(quality['model_coverage'],quality['scan_coverage'])>=.65
        if not full_fit:
            if quality['model_coverage']<.50 or quality['scan_coverage']<.80:continue
            nearest = np.partition(d1, len(d1)//2)[:len(d1)//2]
            rmse=float(np.sqrt((np.mean(nearest**2)+np.mean(d2**2))/2))
            p95=float(max(np.percentile(nearest,95),np.percentile(d2,95)))
            if rmse>.003 or p95>.006:continue
            quality.update(rmse_m=rmse,p95_m=p95,partial_surface=True)
        r3=np.eye(3);r3[:2,:2]=rotation
        transform=np.eye(4);transform[:3,:3]=basis@r3
        transform[:3,3]=origin+basis[:,:2]@translation-transform[:3,:3]@np.array([0.,0.,model[0,2]])
        if table_top is not None:
            transform[2,3] = table_top - vertices[:,2].min()
            quality['supported_top_p95_m'] = support_error
        (accepted if full_fit else partial).append((rmse,transform,quality))
    fits = accepted or partial
    if not fits:raise ValueError('CAD and scan mismatch: no sufficiently covered rigid fit')
    error,transform,quality=min(fits,key=lambda v:v[0])
    if not accepted:
        if reference is None:
            raise ValueError('A partial scan requires an approximate CAD pose')
        comparable = [candidate for candidate in partial if candidate[0] <= error + .00035]
        # A previously displayed orientation can resolve a 180-degree flip.
        # Accept a partial update only near the previous pose.
        comparable = [candidate for candidate in comparable
                      if np.trace(reference[:3,:3].T @ candidate[1][:3,:3])
                      >= 1 + 2*math.cos(math.radians(20))
                      and np.linalg.norm(reference[:3,3]-candidate[1][:3,3])<=.008]
        if not comparable:
            raise ValueError('Partial scan does not confirm the approximate CAD pose; place the model closer or expose more of the part')
        _,transform,quality=min(comparable,key=lambda v:v[0])
        for _, other, _ in comparable:
            angle=np.arccos(np.clip((np.trace(transform[:3,:3].T@other[:3,:3])-1)/2,-1,1))
            if angle>math.radians(8) or np.linalg.norm(transform[:3,3]-other[:3,3])>.008:
                raise ValueError('Partial scan admits several CAD poses; expose a distinct edge or feature')
    quality.update(preliminary=True,execution_ready=False,
                   scope='thin part top surface; underside and hidden geometry are unverified')
    return transform,quality
def voxel_sample(points, spacing):
    _, indices = np.unique(np.floor(points / spacing), axis=0, return_index=True)
    return points[indices]


def sample_supported_surface(vertices, faces):
    a, b, c = vertices[faces].transpose(1, 0, 2)
    area = np.linalg.norm(np.cross(b - a, c - a), axis=1)
    visible = (vertices[faces, 2] > .001).any(axis=1)
    weights = area * visible
    if weights.sum() == 0:
        raise ValueError('CAD has no surface above the support plane')
    rng = np.random.default_rng(41)
    index = rng.choice(len(faces), 45000, p=weights / weights.sum())
    uv = rng.random((len(index), 2))
    reflected = uv.sum(axis=1) > 1
    uv[reflected] = 1 - uv[reflected]
    sampled = a[index] + (b - a)[index] * uv[:, :1] + (c - a)[index] * uv[:, 1:]
    return voxel_sample(sampled[sampled[:, 2] > .002], .0015)


def fit_supported_preview(points_world, vertices, faces, reference, table_top, allow_side_view):
    # The part rests on the table. Only XY and yaw are observable placement variables.
    bottom = vertices[:, 2].min()
    vertices = vertices - [0., 0., bottom]
    height = vertices[:, 2].max()
    scan = np.asarray(points_world, dtype=float) - [0., 0., table_top]
    if scan.ndim != 2 or scan.shape[1] != 3 or not np.isfinite(scan).all():
        raise ValueError('Invalid point cloud for CAD registration')
    scan = voxel_sample(scan[(scan[:, 2] > .002) & (scan[:, 2] < height + .007)], .0025)
    pairs = cKDTree(scan).query_pairs(.007, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
                       shape=(len(scan), len(scan))).tocsr()
    _, labels = connected_components(graph, directed=False)
    elevated_start = max(.012, .3 * height)
    elevated = scan[:, 2] > elevated_start
    if elevated.sum() < 40:
        raise ValueError('Point cloud has too few points above the base for this CAD')
    counts = np.bincount(labels[elevated])
    scan = scan[labels == counts.argmax()]
    high = scan[:, 2] > elevated_start
    if high.sum() < 40 or np.percentile(scan[high, 2], 95) < height - .009:
        raise ValueError('Measured height does not match the selected CAD')
    surface = sample_supported_surface(vertices, faces)
    feature = surface[:, 2] > elevated_start
    tree = cKDTree(surface)
    scan_tree = cKDTree(scan)
    center = np.median(scan[:, :2], axis=0)
    feature_shift = np.median(scan[high, :2], axis=0)
    model_center = np.median(surface[:, :2], axis=0)
    feature_center = np.median(surface[feature, :2], axis=0)
    seeds = []
    reference_yaw = None
    if reference is not None:
        reference_yaw = math.atan2(reference[1, 0], reference[0, 0])
    angles = np.arange(12) * (math.pi / 6)
    if reference_yaw is not None:
        angles = np.append(angles, reference_yaw)
    for angle in angles:
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array([[c, -s], [s, c]])
        seeds.extend((rotation, shift) for shift in (
            center - model_center @ rotation.T, feature_shift - feature_center @ rotation.T))
    if reference is not None:
        seeds.append((rotation, reference[:2, 3]))

    def refine(points, rotation, translation, iterations):
        local = points.copy()
        elevated_weight = 1 + (points[:, 2] > elevated_start)
        for iteration in range(iterations):
            local[:, :2] = (points[:, :2] - translation) @ rotation
            distances, indices = tree.query(local)
            weights = np.maximum(0., 1 - distances / .022) * elevated_weight
            if np.count_nonzero(weights) < 40:
                break
            new_rotation, new_translation = rigid2(surface[indices, :2], points[:, :2], weights)
            converged = (np.linalg.norm(new_translation - translation) < .000025
                         and np.linalg.norm(new_rotation - rotation) < .0003)
            rotation, translation = new_rotation, new_translation
            if converged:
                break
        return rotation, translation, iteration + 1

    coarse = voxel_sample(scan, .005)
    matches = []
    side_matches = []
    local = scan.copy()
    for rotation, translation in seeds:
        rotation, translation, _ = refine(coarse, rotation, translation, 30)
        coarse_local = np.column_stack(((coarse[:, :2] - translation) @ rotation, coarse[:, 2]))
        residual, _ = tree.query(coarse_local)
        if np.mean(residual < .007) < .7:
            continue
        rotation, translation, iterations = refine(scan, rotation, translation, 60)
        local[:, :2] = (scan[:, :2] - translation) @ rotation
        residual, _ = tree.query(local)
        covered = residual < .0045
        if covered.mean() < .80 or covered[high].mean() < .80:
            continue
        moved = surface.copy()
        moved[:, :2] = surface[:, :2] @ rotation.T + translation
        reverse, _ = scan_tree.query(moved)
        coverage = float(np.mean(reverse < .0045))
        feature_coverage = float(np.mean(reverse[feature] < .0045))
        if coverage < .44:
            continue
        rmse = float(np.sqrt(np.mean(np.minimum(residual, .009)**2)))
        side_view = feature_coverage < .40
        if side_view:
            # Occluded faces do not count against a one-sided scan when the
            # visible base and upright surface both agree closely with CAD.
            if (not allow_side_view or feature_coverage < .15 or rmse > .002
                    or min(covered.mean(), covered[high].mean()) < .95):
                continue
        elif rmse > .0035:
            continue
        quality = dict(rmse_m=rmse, p95_m=float(np.percentile(residual, 95)),
                       model_coverage=coverage, elevated_model_coverage=feature_coverage,
                       scan_coverage=float(covered.mean()), elevated_scan_coverage=float(covered[high].mean()),
                       point_count=len(scan), iterations=iterations, partial_surface=True,
                       preliminary=True, execution_ready=False, scope='visible surfaces of an upright part')
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        score = rmse + .002 * (1 - feature_coverage) + .001 * (1 - coverage)
        if side_view:
            quality['side_view'] = True
            side_matches.append((score, yaw, rotation, translation, quality))
        else:
            matches.append((score, yaw, rotation, translation, quality))
    matches = matches or side_matches
    if not matches:
        raise ValueError('Visible scan surfaces do not match the selected 3D CAD')

    def angle_distance(a, b):
        return abs(math.remainder(a - b, 2 * math.pi))

    matches.sort(key=lambda fit: fit[0])
    best = matches[0]
    comparable = [fit for fit in matches if fit[0] <= best[0] + .00035]
    if reference_yaw is not None:
        best = min(comparable, key=lambda fit: angle_distance(fit[1], reference_yaw))
    _, yaw, rotation, translation, quality = best
    for _, other_yaw, _, other_translation, _ in comparable:
        if np.linalg.norm(other_translation - translation) > .008:
            raise ValueError('Partial 3D scan admits several CAD positions')
        if (angle_distance(yaw, other_yaw) > math.radians(12)
                and (reference_yaw is None or
                     abs(angle_distance(other_yaw, reference_yaw) -
                         angle_distance(yaw, reference_yaw)) < math.radians(20))):
            raise ValueError('Partial 3D scan admits several CAD orientations')
    transform = np.eye(4)
    transform[:2, :2] = rotation
    transform[:2, 3] = translation
    transform[2, 3] = table_top - bottom
    return transform, quality
