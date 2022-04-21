
import numpy as np
from math import pi, cos, sin
from firebot_common.constants import MAP_SIZE, BODY_RADIUS

class Robot():

    def __init__(self):
        self.wheel_base = 0.17
        self.wheel_radius = 0.1
        self.body_radius = 0.1013
        self.N_sensors = 8

        self.sensor_offsets = np.array( [0.0091]+[0.1013]*(self.N_sensors-1) )
        #self.sensor_dirs = np.vstack([[cos(phi), sin(phi)] for phi in np.linspace(0, 2*pi, self.N_sensors, endpoint=False)+pi/4])
        self.sensor_dirs = np.vstack([[cos(phi), sin(phi)] for phi in np.linspace(0, 2*pi, self.N_sensors, endpoint=False)])

        self.hits = np.ones(self.N_sensors) * 0.1

        self.heat_fov = 41 * pi / 180
        self.heat_dirs = np.vstack([[cos(phi), sin(phi)] for phi in np.linspace(self.heat_fov/2, -self.heat_fov/2, 8, endpoint=True)])

        posx = np.random.uniform(BODY_RADIUS, MAP_SIZE-BODY_RADIUS)
        posy = np.random.uniform(BODY_RADIUS, MAP_SIZE-BODY_RADIUS)
        self.pos = pos = np.hstack((posx, posy))
        self.angle = np.random.uniform(0.0, 2.0*pi)
    
    @property
    def x(self):
        return self.pos[0]
    
    @property
    def y(self):
        return self.pos[1]
    
    @property
    def dir(self):
        return np.array([cos(self.angle), sin(self.angle)])
    
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
