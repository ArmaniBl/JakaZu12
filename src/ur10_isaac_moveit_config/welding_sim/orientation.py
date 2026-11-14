"""Bounded candidate tilts; the seam and TCP positions are never changed."""
import math
import numpy as np
from .geometry import quaternion, rotation


def tilted_orientation(q, tilt=0., direction=0., roll=0.):
    if not all(math.isfinite(v) for v in (tilt,direction,roll)) or not 0<=tilt<=60:
        raise ValueError('Tilt must be finite and within 0..60 degrees')
    def rz(angle):
        angle=math.radians(angle);c,s=math.cos(angle),math.sin(angle)
        return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    angle=math.radians(tilt);c,s=math.cos(angle),math.sin(angle)
    ry=np.array([[c,0,s],[0,1.,0],[-s,0,c]])
    return quaternion(rotation(q)@rz(roll)@rz(direction)@ry@rz(-direction))


def candidates(tilt, direction, maximum):
    yield tilt,direction,0.
    seen={(round(tilt,8),round(direction%360,8),0.)}
    for inclination in dict.fromkeys((min(30.,maximum),maximum,min(15.,maximum),0.)):
        for heading in (0.,90.,180.,270.):
            for roll in (90.,-90.,180.,0.):
                if inclination==0 and heading!=0:continue
                key=(inclination,heading,roll)
                if key not in seen:
                    seen.add(key);yield key
