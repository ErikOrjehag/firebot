
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
        self.linear = 0
        self.angular = 0
        self.map = Map()
        self.robot = Robot()
        self.keys = key.KeyStateHandler()
        self.renderer = Renderer("Simulation")
        self.renderer.set_map(self.map)
        self.renderer.set_robot(self.robot)
        self.renderer.window.push_handlers(self.keys)

    def update(self, dt):
        self.robot.pos += dt * self.linear * np.array([
            cos(self.robot.angle), sin(self.robot.angle)])
        self.robot.angle += dt * self.angular

        self.robot.hits = calc_hits(self.robot.pos,
                         self.robot.angle,
                         self.robot.sensor_dirs, 
                         self.robot.sensor_offsets, 
                         self.map.walls)
