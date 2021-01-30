
import pyglet
from pyglet.window import mouse, key
import yaml
import numpy as np
from math import cos, sin, pi, sqrt, exp, fabs
from montecarlo import calc_hits, ParticleFilter

class Robot():

    def __init__(self):

        #self.pos = np.array([0.5, 1.0])
        #self.angle = pi/10.
        posx = np.random.uniform(0.1, 2.3)
        posy = np.random.uniform(0.1, 2.3)
        self.pos = pos = np.hstack((posx, posy))
        self.angle = np.random.uniform(0.0, 2.0*pi)

        self.wheel_base = 0.15
        self.wheel_radius = 0.4
        self.body_radius = 0.18/2

        N_sensors = 5
        self.sensor_offsets = np.array( [self.body_radius]*N_sensors )
        #self.sensor_dirs = np.vstack([[cos(phi), sin(phi)] for phi in np.linspace(0, 2*pi, N_sensors, endpoint=False)+pi/4])
        self.sensor_dirs = np.vstack([[cos(phi), sin(phi)] for phi in np.linspace(0, 2*pi, N_sensors, endpoint=False)])
        self.hits = np.zeros(N_sensors)

        self.batch = pyglet.graphics.Batch()
        circle = self.body_radius*np.vstack([[cos(phi), sin(phi)] for phi in np.linspace(0., 2.*pi, 20)])
        triangle = 0.5*self.body_radius*np.vstack([[cos(phi)+0.6, sin(phi)] for phi in np.linspace(0., 2.*pi, 4)])
        
        g1 = pyglet.graphics.Group()
        g2 = pyglet.graphics.Group()

        self.batch.add(len(circle), pyglet.gl.GL_LINE_STRIP, g1,
            ('v2f', circle.flatten()),
        )

        self.batch.add(len(triangle), pyglet.gl.GL_LINE_STRIP, g2,
            ('v2f', triangle.flatten()),
        )

        lines = np.hstack((self.sensor_offsets[...,None]*self.sensor_dirs, (self.sensor_offsets+self.hits)[:,None]*self.sensor_dirs))

        self.lines_gl = self.batch.add(len(lines)*2, pyglet.gl.GL_LINES, None,
            ('v2f', lines.flatten()),
            ('c3B', (255,0,0)*len(lines)*2)
        )

    def draw(self):
        pyglet.gl.glPushMatrix()
        pyglet.gl.glTranslatef(self.pos[0], self.pos[1], 0)
        pyglet.gl.glRotatef(180./pi*self.angle, 0, 0, 1)
        self.batch.draw()
        pyglet.gl.glPopMatrix()

    def calc_hits(self, walls):
        self.hits = calc_hits(self.pos, self.angle, self.sensor_dirs, self.sensor_offsets, walls)
        self.hits += 0.05*np.random.randn(len(self.hits))
        lines = np.hstack((self.sensor_offsets[...,None]*self.sensor_dirs, (self.sensor_offsets+self.hits)[:,None]*self.sensor_dirs))
        self.lines_gl.vertices = lines.flatten()

    def move(self, linear, angular):
        self.pos += np.array([cos(self.angle), sin(self.angle)])*linear
        self.angle += angular

def main():

    robot = Robot()
    keys = key.KeyStateHandler()
    pf = ParticleFilter([
        (50, 0.1, 2.3, 0.1, 2.3),
        #(50, 0.1, 0.8, 0.1, 1.1),
    ])

    def update(dt):
        max_lin = 0.2*3
        max_ang = 1.0*3
        linear = 0.0
        angular = 0.0
        if keys[key.UP] or keys[key.W]:
            linear += max_lin
        if keys[key.DOWN] or keys[key.S]:
            linear -= max_lin
        if keys[key.LEFT] or keys[key.A]:
            angular += max_ang
        if keys[key.RIGHT] or keys[key.D]:
            angular -= max_ang

        """
        linear = max_lin
        if robot.hits[3]<0.2:
            angular = max_ang
            linear = 0.2*max_lin
        elif robot.hits[1]<0.2:
            angular = -max_ang
            linear = 0.2*max_lin
        elif robot.hits[0]<0.2:
            linear = -0.2*max_lin
        """
        
        linear *= dt
        angular *= dt

        robot.move(linear, angular)
        robot.calc_hits(walls)
        pf.update(linear, angular, robot, walls)

    window_width = 600
    window_height = 500
    window = pyglet.window.Window(window_width, window_height)
    
    # Window handlers
    window.push_handlers(keys)

    # Update every 60 times per second
    pyglet.clock.schedule_interval(update, 1 / 60.0)

    batch = pyglet.graphics.Batch()

    verts = []
    walls = []

    with open('map.yaml') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        for v in data['vertices']:
            verts.append(np.array(v)/100.)
        for w in data['walls']:
            walls.append(np.concatenate((verts[w[0]], verts[w[1]])))

    verts = np.vstack(verts)
    walls = np.vstack(walls)

    walls_gl = batch.add(len(walls)*2, pyglet.gl.GL_LINES, None,
        ('v2f', walls.flatten()),
        ('c3B', (255,255,255)*len(walls)*2)
    )

    vertex_gl = batch.add(len(verts), pyglet.gl.GL_POINTS, None,
        ('v2f', verts.flatten()),
        ('c3B', (255,0,0)*len(verts))
    )

    @window.event
    def on_draw():

        window.clear()
        
        pyglet.gl.glPushMatrix()
        pyglet.gl.glRotatef(90, 0, 0, 1)
        pyglet.gl.glTranslatef(window_height/2, -window_width/2, 0)
        pyglet.gl.glScalef(200, 200, 1)
        pyglet.gl.glTranslatef(-2.42/2, -2.42/2, 0)

        batch.draw()

        robot.draw()

        pf.draw()

        pyglet.gl.glPopMatrix()

    pyglet.app.run()

if __name__ == "__main__":
    main()

"""

    

    #event_logger = pyglet.window.event.WindowEventLogger()
    #window.push_handlers(event_logger)

"""