
import numpy as np
from math import cos, sin, pi, sqrt, exp, fabs
from random import random
#from numba import njit

#@njit
def is_wall_collision(pos, radius, walls):
    
    r2 = radius**2
    p0 = walls[:,:2]
    p1 = walls[:,2:]
    v = p1 - p0
    w = pos - p0

    c1 = (w*v).sum(1)
    mask1 = c1 <= 0
    if np.any(mask1):
        d = ((pos - p0[mask1])**2).sum(1)
        if np.any(d <= r2):
            return True
    
    c2 = (v*v).sum(1)
    mask2 = c2 <= c1
    if np.any(mask2):
        d = ((pos - p1[mask2])**2).sum(1)
        if np.any(d <= r2):
            return True

    mask3 = np.logical_not(np.logical_or(mask1, mask2))
    b = c1[mask3] / c2[mask3]
    Pb = p0[mask3] + np.expand_dims(b, 1) * v[mask3]
    d = ((pos - Pb)**2).sum(1)
    if np.any(d <= r2):
        return True
    
    return False


def test_is_wall_collision():
    from mapp import Map
    mapp = Map()
    is_collision = is_wall_collision(np.array([0.0, 0.0]), 0.1, mapp.walls)
    print(is_collision)

def main():
    test_is_wall_collision()

if __name__ == '__main__':
    main()
