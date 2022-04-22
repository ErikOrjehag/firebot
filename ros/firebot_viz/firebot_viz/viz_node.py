
import yaml
import pyglet
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float64MultiArray
from firebot_common.robot import Robot
from firebot_common.renderer import Renderer
from firebot_common.mapp import Map
from firebot_common.montecarlo import ParticleFilter
from firebot_common.constants import MAP_SIZE, BODY_RADIUS
import rclpy.qos

class VizNode(Node):

    def __init__(self):
        super().__init__('viz_node')
        self.map = Map()
        self.robot = Robot()
        self.pf = ParticleFilter([
            (50, BODY_RADIUS, MAP_SIZE-BODY_RADIUS, BODY_RADIUS, MAP_SIZE-BODY_RADIUS),
            #(50, 0.1, 0.8, 0.1, 1.1),
        ])
        self.renderer = Renderer("Vizualisation")
        self.renderer.set_map(self.map)
        self.renderer.set_robot(self.robot)
        self.renderer.set_pf(self.pf)
        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, 1)
        self.pf_sub = self.create_subscription(PoseArray, 'pf', self.pf_callback, 1)
        self.hits_sub = self.create_subscription(Float64MultiArray, 'hits', self.hits_callback, rclpy.qos.qos_profile_sensor_data)
        self.heat_sub = self.create_subscription(Float64MultiArray, 'heat', self.heat_callback, rclpy.qos.qos_profile_sensor_data)
        self.carrot_sub = self.create_subscription(Pose, 'carrot', self.carrot_callback, 1)

    def heat_callback(self, msg):
        self.renderer.set_heat(msg.data)

    def pose_callback(self, msg):
        self.robot.x = msg.position.x
        self.robot.y = msg.position.y
        self.robot.angle = msg.orientation.z

    def carrot_callback(self, msg: Pose):
        self.renderer.set_fire(np.array([msg.position.x, msg.position.y]))

    def hits_callback(self, msg):
        self.robot.hits = np.array(msg.data)
    
    def pf_callback(self, msg):
        for i, pose in enumerate(msg.poses):
            self.pf.pos[i,0] = pose.position.x
            self.pf.pos[i,1] = pose.position.y
            self.pf.angle[i] = pose.orientation.z

def main(args=None):
    rclpy.init(args=args)
    viz_node = VizNode()

    def update(dt):
        rclpy.spin_once(viz_node, timeout_sec=0)

    pyglet.clock.schedule_interval(update, 1 / 60.0)
    
    pyglet.app.run()
    viz_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()