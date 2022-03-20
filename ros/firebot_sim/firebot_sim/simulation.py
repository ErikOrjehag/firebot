
from firebot_common.mapp import Map
from firebot_common.robot import Robot
from firebot_common.renderer import Renderer
from firebot_common.calc_hits import calc_hits
from firebot_common.hybridastar import hybrid_astar_search, smooth_path
from time import time
from math import sin, cos
from pyglet.window import mouse, key
import numpy as np
from multiprocessing import Process, Queue

def thread_target(queue, *args):
    final_node = hybrid_astar_search(*args)
    if final_node is not None:
        smooth_path(final_node)
    queue.put(final_node)

class Simulation:

    def __init__(self):
        self.linear = 0
        self.angular = 0
        self.map = Map()
        self.robot = Robot()
        self.renderer = Renderer("Simulation")
        self.renderer.set_map(self.map)
        self.renderer.set_robot(self.robot)
        self.renderer.set_pf(None) # lol
        self.queue = Queue()
        self.search_result = None
        self.search_process = None

    def update(self, dt):
        self.robot.pos += dt * self.linear * np.array([
            cos(self.robot.angle), sin(self.robot.angle)])
        self.robot.angle += dt * self.angular

        self.robot.hits = calc_hits(self.robot.pos,
                         self.robot.angle,
                         self.robot.sensor_dirs, 
                         self.robot.sensor_offsets, 
                         self.map.walls)

        # if self.search_process is None or not self.search_process.is_alive():
        #     if not self.queue.empty():
        #         path = self.queue.get(False)
        #         self.renderer.set_path(path)
        #     self.search_process = Process(target=thread_target, name='search_process', args=(self.queue, self.map.walls, self.robot.x, self.robot.y, self.robot.angle, 1.6, 1.1, 0))
        #     self.search_process.start()


def main(args=None):
    import pyglet
    simulation = Simulation()
    pyglet.clock.schedule_interval(simulation.update, 1 / 60.0)
    pyglet.app.run()


if __name__ == '__main__':
    main()
