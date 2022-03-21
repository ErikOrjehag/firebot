
import numpy as np
from math import cos, sin, pi, sqrt, exp, fabs
from random import random

def calc_hits(pos, angle, sensor_dirs, sensor_offsets, walls):
    """
	pos: robot position
	angle: robot angle
	sensor_dirs: normalized direction vectors of N sensors
	sensor_offsets: offsets from robot center for the N sensors
	walls: vectors (point_start, point_end) for M walls.
	"""

    pt0 = walls[:,:2,None] - pos[...,None]
    pt1 = walls[:,2:,None] - pos[...,None]

    si = sin(angle)
    co = cos(angle)

    rays = np.array([
        [co, -si],
        [si, co]
    ]) @ sensor_dirs[...,None]

    hits = np.zeros(len(rays))

    w = pt0[:,:,0]
    u = pt1[:,:,0] - pt0[:,:,0]

    denom = (rays[:,0]*u[:,1] - rays[:,1]*u[:,0])
    # Do not devide by zero
    denom = np.where(denom < 0, -1., 1.) * np.maximum(np.abs(denom), 1e-9)
    s = (rays[:,1]*w[:,0] - rays[:,0]*w[:,1]) / denom
    
    p = (w + s[...,None] * u)

    # inside line u
    mask1 = np.logical_and(s >= 0, s <= 1)

    # in front of ray
    mask2 = (p * np.repeat(rays.transpose((0,2,1)), p.shape[1], 1)).sum(2) >= sensor_offsets[...,None]

    mask = np.logical_and(mask1, mask2)

    h = np.linalg.norm(p, axis=2)
    hits = (np.bitwise_not(mask) * 1e9 + h).min(1) - sensor_offsets

    return hits