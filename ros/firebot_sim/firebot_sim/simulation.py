
from firebot_common.mapp import Map
from firebot_common.robot import Robot
from firebot_common.renderer import Renderer
from firebot_common.calc_hits import calc_hits
from firebot_common.hybridastar import hybrid_astar_search, smooth_path
from time import time
from math import sin, cos, acos, pi, atan2
from pyglet.window import mouse, key
import numpy as np
from multiprocessing import Process, Queue
import firebot_common.wall_collision
from firebot_common.dijkstras import dijkstras_search, flow
from firebot_common.constants import BODY_RADIUS

def thread_target(queue, *args):
    ts = time()
    final_node = hybrid_astar_search(*args)
    if final_node is not None:
        smooth_path(final_node)
    queue.put(final_node)
    t = time() - ts
    print(f'Hz: {1.0/t:.0f}, t: {t:.2}s')

class Simulation:

    def __init__(self):
        self.linear = 0
        self.angular = 0
        self.map = Map()
        self.robot = Robot()
        self.robot.pos = np.array((0.413635909861095, 0.6977477692875264))
        self.robot.angle = 0.5
        self.distmap = firebot_common.wall_collision.generate_distance_map(self.map)
        self.goal_x = 1.6
        self.goal_y = 1.1
        self.dijkstras = dijkstras_search(self.distmap, self.goal_x, self.goal_y) + 0.3 * np.clip(1.0 - self.distmap / (BODY_RADIUS*2), 0.0, 1.0)
        self.renderer = Renderer("Simulation")
        self.renderer.set_map(self.map)
        self.renderer.set_distmap(self.distmap)
        self.renderer.set_dijkstras(self.dijkstras)
        self.renderer.set_robot(self.robot)
        self.renderer.set_pf(None) # lol
        self.queue = Queue()
        self.search_result = None
        self.search_process = None

    def update(self, dt):
        # self.robot.pos += dt * self.linear * np.array([
        #     cos(self.robot.angle), sin(self.robot.angle)])
        # self.robot.angle += dt * self.angular

        dir = flow(self.dijkstras, *(self.robot.pos + 0.0 * np.array([cos(self.robot.angle), sin(self.robot.angle)])))
        self.robot.pos += dt * 0.2 * np.array([
            cos(self.robot.angle), sin(self.robot.angle)])
        alpha = acos(dir.dot(np.array([cos(self.robot.angle), sin(self.robot.angle)])))
        if dir[0]*sin(self.robot.angle) - dir[1]*cos(self.robot.angle) > 0:
            alpha *= -1
        
        self.robot.angle += dt * 2.0 * alpha
        
        self.renderer.set_nav_dir(dir)

        self.robot.hits = calc_hits(self.robot.pos,
                         self.robot.angle,
                         self.robot.sensor_dirs, 
                         self.robot.sensor_offsets, 
                         self.map.walls)

        # if self.search_process is None or not self.search_process.is_alive():
        #     if not self.queue.empty():
        #         path = self.queue.get(False)
        #         self.renderer.set_path(path)
        #     self.search_process = Process(target=thread_target, name='search_process', args=(self.queue, self.map.walls, self.distmap, self.dijkstras, self.robot.x, self.robot.y, self.robot.angle, self.goal_x, self.goal_y, 0))
        #     self.search_process.start()


def main(args=None):
    import pyglet
    simulation = Simulation()
    pyglet.clock.schedule_interval(simulation.update, 1 / 60.0)
    pyglet.app.run()


if __name__ == '__main__':
    main()
