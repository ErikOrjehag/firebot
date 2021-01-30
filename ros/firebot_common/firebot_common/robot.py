
import numpy as np
from math import pi, cos, sin

class Robot():

    def __init__(self):
        self.wheel_base = 0.15
        self.wheel_radius = 0.4
        self.body_radius = 0.18/2
        self.N_sensors = 5

        self.sensor_offsets = np.array( [self.body_radius]*self.N_sensors )
        #self.sensor_dirs = np.vstack([[cos(phi), sin(phi)] for phi in np.linspace(0, 2*pi, self.N_sensors, endpoint=False)+pi/4])
        self.sensor_dirs = np.vstack([[cos(phi), sin(phi)] for phi in np.linspace(0, 2*pi, self.N_sensors, endpoint=False)])

        self.hits = np.ones(self.N_sensors) * 0.1

        posx = np.random.uniform(0.1, 2.3)
        posy = np.random.uniform(0.1, 2.3)
        self.pos = pos = np.hstack((posx, posy))
        self.angle = np.random.uniform(0.0, 2.0*pi)
    
    @property
    def x(self):
        return self.pos[0]
    
    @property
    def y(self):
        return self.pos[1]
    
    @x.setter
    def x(self, x):
        self.pos[0] = x
    
    @y.setter
    def y(self, y):
        self.pos[1] = y

    @property
    def hits(self):
        return self._hits

    @hits.setter
    def hits(self, hits):
        self._hits = hits
        self.lines = np.hstack((
            self.sensor_offsets[..., None] * self.sensor_dirs,
           (self.sensor_offsets + hits)[:, None] * self.sensor_dirs ))
