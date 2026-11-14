"""Mass integrals for consistently wound triangular CAD surfaces, SI units."""
import numpy as np


def mesh_mass(triangles, density):
    triangles=np.asarray(triangles,dtype=float)
    if triangles.ndim!=3 or triangles.shape[1:]!=(3,3) or not np.all(np.isfinite(triangles)):
        raise ValueError('Invalid triangles')
    if not np.isfinite(density) or density<=0:raise ValueError('Invalid density')
    a,b,c=triangles[:,0],triangles[:,1],triangles[:,2]
    volumes=np.einsum('ij,ij->i',a,np.cross(b,c))/6
    volume=volumes.sum()
    if abs(volume)<1e-12:raise ValueError('Surface has no reliable enclosed volume')
    sign=1. if volume>0 else -1.
    weights=volumes*sign*density
    mass=weights.sum()
    sums=a+b+c
    center=np.einsum('i,ij->j',weights,sums)/(4*mass)
    outer=lambda x:np.einsum('ni,nj->nij',x,x)
    second=np.einsum('n,nij->ij',weights,outer(a)+outer(b)+outer(c)+outer(sums))/20
    inertia=np.trace(second)*np.eye(3)-second-mass*(center@center*np.eye(3)-np.outer(center,center))
    inertia=(inertia+inertia.T)/2
    if np.linalg.eigvalsh(inertia).min()<=0:raise ValueError('Invalid mesh inertia')
    return float(mass),center,inertia


def combine(parts):
    mass=sum(m for m,_,_ in parts)
    if not np.isfinite(mass) or mass<=0:raise ValueError('Invalid total mass')
    center=sum(m*np.asarray(c) for m,c,_ in parts)/mass
    inertia=np.zeros((3,3))
    for m,c,tensor in parts:
        delta=np.asarray(c)-center
        inertia+=tensor+m*(delta@delta*np.eye(3)-np.outer(delta,delta))
    return mass,center,(inertia+inertia.T)/2
