
import numpy as np
from math import cos, sin, pi, sqrt, exp, fabs
from random import random

def calc_hits(pos, angle, sensor_dirs, sensor_offsets, walls):
    
    pt0 = walls[:,:2,None] - pos[...,None]
    pt1 = walls[:,2:,None] - pos[...,None]

    si = sin(angle)
    co = cos(angle)

    rays = np.array([
        [co, -si],
        [si, co]
    ]) @ sensor_dirs[...,None]

    hits = np.zeros(len(rays))

    for i, ray in enumerate(rays):

        # http://geomalgorithms.com/a05-_intersect-1.html

        w = pt0[:,:,0]
        u = pt1[:,:,0] - pt0[:,:,0]
        v = ray[...,0]

        # Do not devide by zero
        denom = (v[0]*u[:,1] - v[1]*u[:,0])
        denom = np.where(denom < 0, -1., 1.) * np.maximum(np.abs(denom), 1e-9)
        s = (v[1]*w[:,0] - v[0]*w[:,1]) / denom

        p = (w + s[...,None] * u)

        # inside line u
        mask1 = np.logical_and(s >= 0, s <= 1)
        p = p[mask1]

        # in front of raw v
        mask2 = p.dot(v) >= sensor_offsets[i]
        p = p[mask2]

        if len(p):
            h = np.linalg.norm(p, axis=1)
            hits[i] = h.min() - sensor_offsets[i]
        else:
            hits[i] = 1e9
    
    return hits