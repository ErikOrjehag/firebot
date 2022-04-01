
import numpy as np
from math import cos, sin, pi, sqrt, exp, fabs
from random import random
from firebot_common.constants import N_DIST_CELLS, DIST_CELL_SIZE

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

def closest_wall_distance(pos, walls):
    p0 = walls[:,:2]
    p1 = walls[:,2:]
    v = p1 - p0
    w = pos - p0

    dd = []

    c1 = (w*v).sum(1)
    mask1 = c1 <= 0
    if np.any(mask1):
        d = ((pos - p0[mask1])**2).sum(1)
        dd.extend(d)
    
    c2 = (v*v).sum(1)
    mask2 = c2 <= c1
    if np.any(mask2):
        d = ((pos - p1[mask2])**2).sum(1)
        dd.extend(d)

    mask3 = np.logical_not(np.logical_or(mask1, mask2))
    b = c1[mask3] / c2[mask3]
    Pb = p0[mask3] + np.expand_dims(b, 1) * v[mask3]
    d = ((pos - Pb)**2).sum(1)
    dd.extend(d)
    
    return np.sqrt(np.array(dd).min())


def generate_distance_map(mapp):
    distances = np.zeros((N_DIST_CELLS, N_DIST_CELLS))
    for r in range(N_DIST_CELLS):
        for c in range(N_DIST_CELLS):
            distances[r, c] = closest_wall_distance(np.array([c * DIST_CELL_SIZE, r * DIST_CELL_SIZE]), mapp.walls)
    return distances

def to_image(distances, maximum=None):
    n = distances.shape[0]
    fmt = 'RGBA'
    data = np.zeros((n, n, 4), np.float64)
    data[...,3] = 1.0
    data[...,0] = np.clip(distances / (maximum if maximum else distances.max()), 0.0, 1.0)
    data[...,1] = data[...,0]
    data[...,2] = data[...,0]
    data = (data.flatten() * 255).astype('uint8')
    data = (GLubyte * data.size)( *data)
    pic = pyglet.image.ImageData(n, n, fmt, data, n*len(fmt))
    pic.anchor_x = 0
    pic.anchor_y = 0
    return pic


from pyglet.gl import *

def main():
    from mapp import Map
    import pyglet
    mapp = Map()
    distances = generate_distance_map(mapp)
    pic = to_image(distances)
    window = pyglet.window.Window(600, 500, "Test")
    def on_draw():
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, pic.get_texture().id)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        window.clear()

        pyglet.gl.glPushMatrix()
        pyglet.gl.glScalef(200*DIST_CELL_SIZE, 200*DIST_CELL_SIZE, 1)
        pic.blit(0, 0, 0)

        pyglet.gl.glPopMatrix()

    window.on_draw = on_draw
    pyglet.app.run()


if __name__ == '__main__':
    main()
