
import pyglet
from pyglet import shapes
from math import pi
import numpy as np
from math import cos, sin
from contextlib import contextmanager
from firebot_common.hybridastar import CELL_SIZE, N_CELLS

@contextmanager
def transform(x, y, angle):
    try:
        pyglet.gl.glPushMatrix()
        pyglet.gl.glTranslatef(x, y, 0)
        pyglet.gl.glRotatef(180./pi*angle, 0, 0, 1)
        yield None
    finally:
        pyglet.gl.glPopMatrix()

class Renderer:

    def __init__(self, title="Title"):
        self.WIDTH = 600
        self.HEIGHT = 500
        self.window = pyglet.window.Window(self.WIDTH, self.HEIGHT, title)
        self.window.on_draw = self.on_draw
        self.map = None
        self.map_b = None
        self.robot = None
        self.path = None
        self.robot_b = None
        self.pf = None
        self.pf1_b = None
        self.pf2_b = None
        self.pf3_b = None
        grid_line = np.array([[0, 0, N_CELLS*CELL_SIZE, 0]])
        self.grid_b = pyglet.graphics.Batch()
        self.grid_b.add(len(grid_line)*2, pyglet.gl.GL_LINES, None,
            ('v2f', grid_line.flatten()),
            ('c3B', (30,30,30)*len(grid_line)*2))

    def set_map(self, mapp):
        self.map = mapp
        self.map_b = pyglet.graphics.Batch()
        self.map_b.add(len(mapp.walls)*2, pyglet.gl.GL_LINES, None,
            ('v2f', mapp.walls.flatten()),
            ('c3B', (255,255,255)*len(mapp.walls)*2))
        self.map_b.add(len(mapp.verts), pyglet.gl.GL_POINTS, None,
            ('v2f', mapp.verts.flatten()),
            ('c3B', (255,0,0)*len(mapp.verts)))
    
    def set_robot(self, robot):
        self.robot = robot
        self.robot_b = pyglet.graphics.Batch()
        circle = robot.body_radius*np.vstack([[cos(phi), sin(phi)] for phi in np.linspace(0., 2.*pi, 20)])
        triangle = 0.5*robot.body_radius*np.vstack([[cos(phi)+0.6, sin(phi)] for phi in np.linspace(0., 2.*pi, 4)])
        g1 = pyglet.graphics.Group()
        g2 = pyglet.graphics.Group()
        self.robot_b.add(len(circle), pyglet.gl.GL_LINE_STRIP, g1,
            ('v2f', circle.flatten()))
        self.robot_b.add(len(triangle), pyglet.gl.GL_LINE_STRIP, g2,
            ('v2f', triangle.flatten()))
        self.robot_lines_gl = self.robot_b.add(len(robot.lines)*2, pyglet.gl.GL_LINES, None,
            ('v2f', robot.lines.flatten()),
            ('c3B', (255,0,0)*len(robot.lines)*2))

    def set_pf(self, pf):
        self.pf = pf
        self.pf1_b = pyglet.graphics.Batch()
        self.pf2_b = pyglet.graphics.Batch()
        self.pf3_b = pyglet.graphics.Batch()
        shape = np.array([ [2, 0], [1, 0], [0, 1], [-1, 0], [0, -1], [1, 0], [0, 0] ])
        self.pf1_b.add(len(shape), pyglet.gl.GL_LINE_STRIP, None,
            ('v2f', 0.03*shape.flatten()),
            ('c3B', (255,255,0)*len(shape)))
        self.pf2_b.add(len(shape), pyglet.gl.GL_LINE_STRIP, None,
            ('v2f', 0.2*shape.flatten()),
            ('c3B', (0,255,0)*len(shape)))
        self.pf3_b.add(len(shape), pyglet.gl.GL_LINE_STRIP, None,
            ('v2f', 0.2*shape.flatten()),
            ('c3B', (0,0,255)*len(shape)))

    def set_path(self, node):
        self.path = node

    def on_draw(self):
        self.window.clear()
        pyglet.gl.glPushMatrix()
        pyglet.gl.glRotatef(90, 0, 0, 1)
        pyglet.gl.glTranslatef(self.HEIGHT/2, -self.WIDTH/2, 0)
        pyglet.gl.glScalef(200, 200, 1)
        pyglet.gl.glTranslatef(-2.42/2, -2.42/2, 0)

        for i in range(N_CELLS):
            with transform(0, i*CELL_SIZE, 0):
                self.grid_b.draw()
            with transform(i*CELL_SIZE, 0, pi/2.0):
                self.grid_b.draw()
        
        if self.map_b is not None:
            self.map_b.draw()
        
        if self.robot is not None:
            self.robot_lines_gl.vertices = self.robot.lines.flatten()
            with transform(self.robot.x, self.robot.y, self.robot.angle):
                self.robot_b.draw()
        
        if self.pf is not None:
            for i in range(self.pf.N):
                with transform(self.pf.pos[i,0], self.pf.pos[i,1], self.pf.angle[i]):
                    self.pf1_b.draw()
            if self.pf.best_pos is not None:
                with transform(self.pf.best_pos[0], self.pf.best_pos[1], self.pf.best_angle):
                    self.pf2_b.draw()
            if self.pf.kf_pos is not None:
                with transform(self.pf.kf_pos[0], self.pf.kf_pos[1], self.pf.kf_angle):
                    self.pf3_b.draw()

        if self.path is not None:
            batch = pyglet.graphics.Batch()
            lines = []
            for i, p in enumerate(self.path):
                if i != 0:
                    p0 = self.path[i-1]
                    lines.append(shapes.Line(p0[0], p0[1], p[0], p[1], 0.02, color=(255, int(255-255*float(i+1)/len(self.path)), 0), batch=batch))
                    #lines[-1].opacity = 100.0-80.0*float(i)/float(len(self.path))
                    lines[-1].opacity = 100
                with transform(p[0], p[1], p[2]):
                    self.pf1_b.draw()
            batch.draw()

        pyglet.gl.glPopMatrix()

def main():
    from hybridastar import hybrid_astar_search
    from wall_collision import is_wall_collision
    from mapp import Map
    from robot import Robot
    import time
    import math
    mapp = Map()
    renderer = Renderer("Test Renderer")
    renderer.set_map(mapp)
    # ts = time.time()
    # #final_node = hybrid_astar_search(mapp.walls, 0.8, 2.0, -3.1415/4, 1.6, 1.1, 3.1415)
    # final_node = hybrid_astar_search(mapp.walls, 0.1, 0.9, -3.1415/100, 0.1, 0.85, pi)
    # print(f'{time.time() - ts:.3f} seconds')
    # renderer.set_pf(None) # lol
    # renderer.set_path(final_node)
    
    robot = Robot()
    renderer.set_robot(robot)

    def update(dt):
        robot.x = 1.0 + math.sin(time.time())
        robot.y = 1.0 + math.cos(time.time())
        is_collision = is_wall_collision(robot.pos, robot.body_radius, mapp.walls)
        print(is_collision)

    pyglet.clock.schedule_interval(update, 1 / 60.0)
    
    pyglet.app.run()

if __name__ == '__main__':
    main()
