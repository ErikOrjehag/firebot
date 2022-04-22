
import pyglet
from pyglet import shapes
from math import pi
import numpy as np
from math import cos, sin, atan2
from contextlib import contextmanager
from firebot_common.constants import DIST_CELL_SIZE, SEARCH_CELL_SIZE, N_SEARCH_CELLS, MAP_SIZE, BODY_RADIUS
import firebot_common.wall_collision

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
        self.WINDOW_SIZE = 600
        self.window = pyglet.window.Window(self.WINDOW_SIZE, self.WINDOW_SIZE, title)
        self.window.on_draw = self.on_draw
        self.map = None
        self.distmap_pic = None
        self.dijkstras_pic = None
        self.heat = None
        self.heat_b = None
        self.heat_r = None
        self.fire = None
        self.fire_b = None
        self.map_b = None
        self.robot = None
        self.path = None
        self.robot_b = None
        self.nav_dir = None
        self.nav_dir_b = None
        self.pf = None
        self.pf1_b = None
        self.pf2_b = None
        self.pf3_b = None
        grid_line = np.array([[0, 0, N_SEARCH_CELLS*SEARCH_CELL_SIZE, 0]])
        self.grid_b = pyglet.graphics.Batch()
        self.grid_b.add(len(grid_line)*2, pyglet.gl.GL_LINES, None,
            ('v2f', grid_line.flatten()),
            ('c3B', (30,30,30)*len(grid_line)*2))

    def set_map(self, mapp):
        self.map = mapp
        self.map_b = pyglet.graphics.Batch()
        self.map_b.add(len(mapp.walls)*2, pyglet.gl.GL_LINES, None,
            ('v2f', mapp.walls.flatten()),
            ('c3B', (0,0,255)*len(mapp.walls)*2))
        self.map_b.add(len(mapp.verts), pyglet.gl.GL_POINTS, None,
            ('v2f', mapp.verts.flatten()),
            ('c3B', (255,0,0)*len(mapp.verts)))
    
    def set_nav_dir(self, nav_dir):
        self.nav_dir = nav_dir
        self.nav_dir_b = pyglet.graphics.Batch()
        triangle = 0.3*np.vstack([[cos(phi)+0.6, sin(phi)] for phi in np.linspace(0., 2.*pi, 4)])
        self.nav_dir_b.add(len(triangle), pyglet.gl.GL_LINE_STRIP, None,
            ('v2f', triangle.flatten()))

    def set_fire(self, fire):
        self.fire = fire
        if self.fire_b is None:
            circle = 0.03*np.vstack([[cos(phi), sin(phi)] for phi in np.linspace(0., 2.*pi, 20)])
            g1 = pyglet.graphics.Group()
            self.fire_b = pyglet.graphics.Batch()
            self.fire_b.add(len(circle), pyglet.gl.GL_LINE_STRIP, g1,
                ('v2f', circle.flatten()),
                ('c3B', (255,150,0)*len(circle)))

    def set_heat(self, heat):
        if self.heat_b is None:
            self.heat_b = pyglet.graphics.Batch()
            self.heat_r = shapes.Rectangle(0.0,0.0,50.0,50.0,color=(255,0,0),batch=self.heat_b)
        self.heat = heat

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
        heat_lines = np.hstack((
            BODY_RADIUS * 0.1 * robot.heat_dirs,
            BODY_RADIUS * 1.2 * robot.heat_dirs ))
        self.robot_b.add(len(heat_lines)*2, pyglet.gl.GL_LINES, None,
            ('v2f', heat_lines.flatten()),
            ('c3B', (0,255,0)*len(heat_lines)*2))
        self.sensor_lines_gl = self.robot_b.add(len(robot.lines)*2, pyglet.gl.GL_LINES, None,
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

    def set_distmap(self, distmap):
        self.distmap_pic = firebot_common.wall_collision.to_image(distmap)
    
    def set_dijkstras(self, dijkstras):
        self.dijkstras_pic = firebot_common.wall_collision.to_image(dijkstras, maximum=4.0)

    def set_path(self, node):
        self.path = node

    def on_draw(self):
        self.window.clear()

        if self.heat is not None:
            pyglet.gl.glPushMatrix()
            for i in range(len(self.heat)):
                    with transform(1+51*i, 1, 0):
                        self.heat_r.opacity = 255 * np.clip(self.heat[i] / 100, 0, 1)
                        self.heat_b.draw()
            pyglet.gl.glPopMatrix()

        pyglet.gl.glPushMatrix()
        pyglet.gl.glRotatef(90, 0, 0, 1)
        pyglet.gl.glTranslatef(self.WINDOW_SIZE/2, -self.WINDOW_SIZE/2, 0)
        pyglet.gl.glScalef(0.8 * self.WINDOW_SIZE / MAP_SIZE, 0.8 * self.WINDOW_SIZE / MAP_SIZE, 1)
        pyglet.gl.glTranslatef(-MAP_SIZE/2, -MAP_SIZE/2, 0)

        for i in range(N_SEARCH_CELLS):
            with transform(0, i*SEARCH_CELL_SIZE, 0):
                self.grid_b.draw()
            with transform(i*SEARCH_CELL_SIZE, 0, pi/2.0):
                self.grid_b.draw()
        
        if self.distmap_pic is not None:
            pyglet.gl.glPushMatrix()
            #pyglet.gl.glEnable(pyglet.gl.GL_TEXTURE_2D)
            #pyglet.gl.glBindTexture(pyglet.gl.GL_TEXTURE_2D, self.distmap_pic.get_texture().id)
            pyglet.gl.glTexParameteri(pyglet.gl.GL_TEXTURE_2D, pyglet.gl.GL_TEXTURE_MAG_FILTER, pyglet.gl.GL_NEAREST)
            pyglet.gl.glScalef(DIST_CELL_SIZE, DIST_CELL_SIZE, 1)
            self.distmap_pic.blit(0, 0, 0)
            pyglet.gl.glPopMatrix()

        if self.dijkstras_pic is not None:
            pyglet.gl.glPushMatrix()
            #pyglet.gl.glEnable(pyglet.gl.GL_TEXTURE_2D)
            #pyglet.gl.glBindTexture(pyglet.gl.GL_TEXTURE_2D, self.dijkstras_pic.get_texture().id)
            pyglet.gl.glTexParameteri(pyglet.gl.GL_TEXTURE_2D, pyglet.gl.GL_TEXTURE_MAG_FILTER, pyglet.gl.GL_NEAREST)
            pyglet.gl.glScalef(SEARCH_CELL_SIZE, SEARCH_CELL_SIZE, 1)
            self.dijkstras_pic.blit(0, 0, 0)
            pyglet.gl.glPopMatrix()

        if self.map_b is not None:
            self.map_b.draw()
        
        if self.fire is not None:
            with transform(self.fire[0], self.fire[1], 0.0):
                self.fire_b.draw()

        if self.robot is not None:
            self.sensor_lines_gl.vertices = self.robot.lines.flatten()
            with transform(self.robot.x, self.robot.y, self.robot.angle):
                self.robot_b.draw()
            if self.nav_dir is not None:
                with transform(self.robot.x, self.robot.y, atan2(self.nav_dir[1], self.nav_dir[0])):
                    self.nav_dir_b.draw()
        
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
    from wall_collision import is_wall_collision, generate_distance_map
    from mapp import Map
    from robot import Robot
    import time
    import math
    
    mapp = Map()
    
    renderer = Renderer("Test Renderer")
    renderer.set_map(mapp)
    
    robot = Robot()
    renderer.set_robot(robot)

    distmap = generate_distance_map(mapp)
    
    renderer.set_distmap(distmap)

    def update(dt):
        robot.x = 1.5 + 0.5*math.sin(0.2*time.time())
        robot.y = 1.5 + 0.5*math.cos(0.2*time.time())
        is_collision = is_wall_collision(robot.pos, robot.body_radius, mapp.walls)
        is_distmap_collision = distmap[int(robot.pos[1] / wall_collision.CELL_SIZE), int(robot.pos[0] / wall_collision.CELL_SIZE)] < robot.body_radius
        print(is_collision, is_distmap_collision)

    pyglet.clock.schedule_interval(update, 1 / 60.0)
    
    pyglet.app.run()

if __name__ == '__main__':
    main()
