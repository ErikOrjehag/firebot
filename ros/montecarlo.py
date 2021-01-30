
import pyglet
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

def normalize_weights(w):
    N = len(w)
    wsum = np.sum(w)
    if wsum > 0.0:
        w /= wsum
    else:
        w = np.ones(N) / N
    return w

class ParticleFilter():

    def __init__(self, initialAreas):
        self.N = 0

        self.noise = 0.05

        self.pos = np.empty((0, 2))

        for n, xmin, xmax, ymin, ymax in initialAreas:
            self.N += n
            posx = np.random.uniform(xmin, xmax, size=(n,1))
            posy = np.random.uniform(ymin, ymax, size=(n,1))
            pos = np.hstack((posx, posy))
            self.pos = np.concatenate((self.pos, pos), axis=0)

        self.angle = np.random.uniform(0.0, 2.0*pi, size=self.N)

        self.w = np.ones(self.N) / self.N

        self.best_pos = None
        self.best_angle = None

        self.kf_pos = None
        self.kf_angle = None

        # Drawing stuff
        self.batch1 = pyglet.graphics.Batch()
        self.batch2 = pyglet.graphics.Batch()
        self.batch3 = pyglet.graphics.Batch()

        shape = np.array([ [2, 0], [1, 0], [0, 1], [-1, 0], [0, -1], [1, 0], [0, 0] ])

        self.batch1.add(len(shape), pyglet.gl.GL_LINE_STRIP, None,
            ('v2f', 0.03*shape.flatten()),
            ('c3B', (255,255,0)*len(shape))
        )

        self.batch2.add(len(shape), pyglet.gl.GL_LINE_STRIP, None,
            ('v2f', 0.2*shape.flatten()),
            ('c3B', (0,255,0)*len(shape))
        )

        self.batch3.add(len(shape), pyglet.gl.GL_LINE_STRIP, None,
            ('v2f', 0.2*shape.flatten()),
            ('c3B', (0,0,255)*len(shape))
        )

    def draw(self):
        # Particles
        for i in range(self.N):
            pyglet.gl.glPushMatrix()
            pyglet.gl.glTranslatef(self.pos[i,0], self.pos[i,1], 0)
            pyglet.gl.glRotatef(180./pi*self.angle[i], 0, 0, 1)
            self.batch1.draw()
            pyglet.gl.glPopMatrix()
        # Best estimate
        if self.best_pos is not None:
            pyglet.gl.glPushMatrix()
            pyglet.gl.glTranslatef(self.best_pos[0], self.best_pos[1], 0)
            pyglet.gl.glRotatef(180./pi*self.best_angle, 0, 0, 1)
            self.batch2.draw()
            pyglet.gl.glPopMatrix()
        if self.kf_pos is not None:
            pyglet.gl.glPushMatrix()
            pyglet.gl.glTranslatef(self.kf_pos[0], self.kf_pos[1], 0)
            pyglet.gl.glRotatef(180./pi*self.kf_angle, 0, 0, 1)
            self.batch3.draw()
            pyglet.gl.glPopMatrix()

    def update(self, linear, angular, robot, walls):

        self.pos += linear*np.vstack((np.cos(self.angle), np.sin(self.angle))).T
        self.angle += angular

        if np.linalg.norm(linear) > 1e-9 or fabs(angular) > 1e-9:
            for i in range(self.N):
                hits = calc_hits(self.pos[i], self.angle[i], robot.sensor_dirs, robot.sensor_offsets, walls)
                self.w[i] += 0.1*(np.prod(1./(self.noise*sqrt(2*pi)) * np.exp(-0.5*((hits-robot.hits)/self.noise)**2)) - self.w[i])
            self.w = normalize_weights(self.w)

            best_i = np.argmax(self.w)
            self.best_pos = self.pos[best_i]
            self.best_angle = self.angle[best_i]

            # Resample
            inds_sort = np.argsort(self.w)
            n1 = int(self.N*0.2)
            n2 = int(self.N*0.3)
            inds_redo = inds_sort[:n1]      # Resample uniformly
            inds_bad = inds_sort[n1:n1+n2]  # Resample normal gauss around good
            inds_good = inds_sort[n1+n2:]   # Keep as is

            N_redo = len(inds_redo)
            N_bad = len(inds_bad)
            N_good = len(inds_good)

            w_good = normalize_weights(self.w[inds_good])

            bins = np.cumsum(np.concatenate((np.zeros(1), w_good)))
            rands = np.random.uniform(0.0, 0.999, N_bad)
            inds_bin = np.digitize(rands, bins)

            for i in range(N_bad):
                good_i = inds_good[inds_bin[i]-1]
                bad_i = inds_bad[i]
                self.pos[bad_i] = self.pos[good_i]+0.03*np.random.randn(2)
                self.angle[bad_i] = self.angle[good_i]+pi/15.*np.random.randn(1)
            
            # Redo
            posx = np.random.uniform(0.1, 2.3, size=(N_redo,1))
            posy = np.random.uniform(0.1, 2.3, size=(N_redo,1))
            self.pos[inds_redo] = pos = np.hstack((posx, posy))
            self.angle[inds_redo] = np.random.uniform(0.0, 2.0*pi, size=N_redo)

        # Kalman filter
        if self.best_pos is not None:
            if self.kf_pos is None:
                self.kf_pos = self.best_pos.copy()
                self.kf_angle = self.best_angle.copy()
            else:
                err_pos = self.best_pos - self.kf_pos
                err_angle = (self.best_angle - self.kf_angle + pi) % (2*pi) - pi

                kf_pos_w = 0.02
                kf_angle_w = 0.02
                self.kf_pos += kf_pos_w * err_pos + (1.0-kf_pos_w) * linear*np.array([cos(self.kf_angle), sin(self.kf_angle)])
                self.kf_angle += kf_angle_w * err_angle + (1.0-kf_angle_w) * angular

"""

wsum = 0.0

for p in particles:
    p.robot.calc_hits(walls)

    gtot = 1.0
    for i in range(4):
        noise = 0.05
        g = 1./(noise*sqrt(2*pi)) * exp(-0.5*((p.robot.hits[i]-robot.hits[i])/noise)**2)
        #gtot *= g
        gtot += g

    p.w = gtot
    wsum += p.w

for p in particles:
    p.w /= wsum

print([p.w for p in particles])
"""