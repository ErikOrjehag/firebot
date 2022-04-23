
import numpy as np
from math import pi, fabs, sqrt
from firebot_common.calc_hits import calc_hits
from firebot_common.constants import MAP_SIZE, BODY_RADIUS, SEARCH_CELL_SIZE, N_SEARCH_CELLS

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

        self.initialAreas = initialAreas
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

        self.dijkstras = None
        self.confidence = 0.0
        self.initialized = False

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

            self.confidence += 0.02 * (w_good.sum() - self.confidence)
            if self.confidence > 0.90:
                self.initialized = True

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
            new = np.empty((0, 2))
            s = 0
            if not self.initialized:
                for i, (n, xmin, xmax, ymin, ymax) in enumerate(self.initialAreas):
                    if i == len(self.initialAreas) - 1:
                        N = N_redo - s
                    else:
                        N = int((n / len(self.w))*N_redo)
                    s += N
                    posx = np.random.uniform(xmin, xmax, size=(N,1))
                    posy = np.random.uniform(ymin, ymax, size=(N,1))
                    pos = np.hstack((posx, posy))
                    new = np.concatenate((new, pos), axis=0)
            else:
                if self.dijkstras is not None:
                    N_redo_left = N_redo
                    i = 0
                    while N_redo_left > 0:
                        i += 1
                        posx = np.random.uniform(BODY_RADIUS, MAP_SIZE - BODY_RADIUS, size=(N_redo_left,1))
                        posy = np.random.uniform(BODY_RADIUS, MAP_SIZE - BODY_RADIUS, size=(N_redo_left,1))
                        r = (posy / SEARCH_CELL_SIZE).astype(int)[:,0]
                        c = (posx / SEARCH_CELL_SIZE).astype(int)[:,0]
                        mask = np.logical_and.reduce((
                            r >= 0,
                            r < N_SEARCH_CELLS,
                            c >= 0,
                            c < N_SEARCH_CELLS,
                            self.dijkstras[r, c] < 3.8
                        ), dtype=bool)
                        
                        pos = np.hstack((posx, posy))
                        pos = pos[mask]
                        N = sum(mask)
                        if N:
                            new = np.concatenate((new, pos), axis=0)
                            N_redo_left -= N
                    # print(i)
                else:
                    posx = np.random.uniform(BODY_RADIUS, MAP_SIZE - BODY_RADIUS, size=(N_redo,1))
                    posy = np.random.uniform(BODY_RADIUS, MAP_SIZE - BODY_RADIUS, size=(N_redo,1))
                    new = np.hstack((posx, posy))

            self.pos[inds_redo] = new
            self.angle[inds_redo] = np.random.uniform(0.0, 2.0*pi, size=N_redo)
