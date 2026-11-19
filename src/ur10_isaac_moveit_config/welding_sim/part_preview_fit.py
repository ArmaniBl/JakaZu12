"""Automatic thin-part visual registration. No robot calibration or motion."""
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def top_surface(vertices_mm,triangles,spacing=.0015):
    vertices=np.asarray(vertices_mm,dtype=float)*.001
    faces=np.asarray(triangles)
    if (vertices.ndim!=2 or vertices.shape[1]!=3 or not 4<=len(vertices)<=100000
            or faces.ndim!=2 or faces.shape[1]!=3 or not 4<=len(faces)<=20000
            or faces.dtype.kind not in 'iu' or faces.min()<0 or faces.max()>=len(vertices)
            or not np.isfinite(vertices).all()):
        raise ValueError('Invalid workpiece mesh')
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
    if s[1]/math.sqrt(len(selected))<.008:raise ValueError('Measured surface is too narrow')
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


def fit_preview(points_world,vertices_mm,triangles):
    vertices,model=top_surface(vertices_mm,triangles)
    scan,origin,basis=dominant_plane(points_world)
    target=(scan-origin)@basis[:,:2]
    # Uniform samples prevent a densely reconstructed corner dominating the fit.
    _,indices=np.unique(np.floor(target/.001),axis=0,return_index=True);target=target[indices]
    source=model[:,:2]
    observed=np.ptp(np.percentile(target,[2,98],axis=0),axis=0)
    expected=np.sort(np.ptp(vertices[:,:2],axis=0))[::-1]
    if np.any(np.sort(observed)[::-1]/expected<.55) or np.any(np.sort(observed)[::-1]/expected>1.45):
        raise ValueError('Dominant scan surface dimensions do not match the selected CAD')
    tree=cKDTree(target);center=(np.percentile(target,2,axis=0)+np.percentile(target,98,axis=0))/2
    base=0. if np.ptp(source[:,0])>np.ptp(source[:,1]) else -math.pi/2
    candidates=[]
    for angle in [base,base+math.pi,base-.08,base+.08,base+math.pi-.08,base+math.pi+.08]:
        c,s=math.cos(angle),math.sin(angle);rotation=np.array([[c,-s],[s,c]])
        moved=source@rotation.T
        translation=center-(moved.min(0)+moved.max(0))/2
        for iteration in range(70):
            moved=source@rotation.T+translation
            d1,j1=tree.query(moved);d2,j2=cKDTree(moved).query(target)
            keep1=d1<.008;keep2=d2<.008
            if keep1.mean()<.5 or keep2.mean()<.5:break
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
        if rmse>.004 or quality['p95_m']>.008 or min(quality['model_coverage'],quality['scan_coverage'])<.65:continue
        r3=np.eye(3);r3[:2,:2]=rotation
        transform=np.eye(4);transform[:3,:3]=basis@r3
        transform[:3,3]=origin+basis[:,:2]@translation-transform[:3,:3]@np.array([0.,0.,model[0,2]])
        candidates.append((rmse,transform,quality))
    if not candidates:raise ValueError('CAD and scan mismatch: no sufficiently covered rigid fit')
    _,transform,quality=min(candidates,key=lambda v:v[0])
    quality.update(preliminary=True,execution_ready=False,
                   scope='thin part top surface; underside and hidden geometry are unverified')
    return transform,quality
