
import numpy as np
from math import pi, fabs, sqrt
from firebot_common.calc_hits import calc_hits
from firebot_common.constants import MAP_SIZE, BODY_RADIUS

def normalize_weights(w):
    w += 0.01
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

        self.confidence = 0.0

    def update(self, linear, angular, robot, walls):
        
        if True or np.linalg.norm(linear) > 1e-9 or fabs(angular) > 1e-9:
            for i in range(self.N):
                hits = calc_hits(self.pos[i], self.angle[i], robot.sensor_dirs, robot.sensor_offsets, walls)
                self.w[i] += 0.5*(np.prod(1./(self.noise*sqrt(2*pi)) * np.exp(-0.5*((hits-robot.hits)/self.noise)**2)) - self.w[i])
            self.w = normalize_weights(self.w)

            best_i = np.argmax(self.w)
            self.best_pos = self.pos[best_i]
            self.best_angle = self.angle[best_i]

            # Resample
            inds_sort = np.argsort(self.w)
            uni = (1.0 - self.confidence)
            n1 = int(self.N*0.6*uni)
            n2 = int(self.N*0.6*(1-uni))
            inds_redo = inds_sort[:n1]      # Resample uniformly
            inds_bad = inds_sort[n1:n1+n2]  # Resample normal gauss around good
            inds_good = inds_sort[n1+n2:]   # Keep as is

            N_redo = len(inds_redo)
            N_bad = len(inds_bad)
            N_good = len(inds_good)

            w_good = self.w[inds_good]

            self.confidence += 0.1 * (w_good.sum() - self.confidence)

            w_good_norm = normalize_weights(w_good)

            bins = np.cumsum(np.concatenate((np.zeros(1), w_good_norm)))
            rands = np.random.uniform(0.0, 0.999, N_bad)
            inds_bin = np.digitize(rands, bins)

            for i in range(N_bad):
                good_i = inds_good[inds_bin[i]-1]
                bad_i = inds_bad[i]
                self.pos[bad_i] = self.pos[good_i]+0.05*np.random.randn(2)
                self.angle[bad_i] = self.angle[good_i]+20.*pi/180.*np.random.randn(1)
            
            # Redo
            posx = np.random.uniform(BODY_RADIUS, MAP_SIZE - BODY_RADIUS, size=(N_redo,1))
            posy = np.random.uniform(BODY_RADIUS, MAP_SIZE - BODY_RADIUS, size=(N_redo,1))
            self.pos[inds_redo] = pos = np.hstack((posx, posy))
            self.angle[inds_redo] = np.random.uniform(0.0, 2.0*pi, size=N_redo)
