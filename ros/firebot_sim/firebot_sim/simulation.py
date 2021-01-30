
from firebot_common.mapp import Map
from firebot_common.robot import Robot
from firebot_common.renderer import Renderer
from firebot_common.calc_hits import calc_hits
from time import time
from math import sin, cos
from pyglet.window import mouse, key
import numpy as np

class Simulation:

    def __init__(self):
        self.map = Map()
        self.robot = Robot()
        self.keys = key.KeyStateHandler()
        self.renderer = Renderer("Simulation")
        self.renderer.set_map(self.map)
        self.renderer.set_robot(self.robot)
        self.renderer.window.push_handlers(self.keys)

    def update(self, dt):
        max_lin = 0.6
        max_ang = 3.0
        linear = 0.0
        angular = 0.0
        if self.keys[key.UP] or self.keys[key.W]:
            linear += max_lin
        if self.keys[key.DOWN] or self.keys[key.S]:
            linear -= max_lin
        if self.keys[key.LEFT] or self.keys[key.A]:
            angular += max_ang
        if self.keys[key.RIGHT] or self.keys[key.D]:
            angular -= max_ang
        self.robot.pos += dt * linear * np.array([
            cos(self.robot.angle), sin(self.robot.angle)])
        self.robot.angle += dt * angular

        self.robot.hits = calc_hits(self.robot.pos,
                         self.robot.angle,
                         self.robot.sensor_dirs, 
                         self.robot.sensor_offsets, 
                         self.map.walls)
