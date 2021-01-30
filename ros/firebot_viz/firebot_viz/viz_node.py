
import yaml
import pyglet
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from firebot_common.robot import Robot
from firebot_common.renderer import Renderer
from firebot_common.mapp import Map

class VizNode(Node):

    def __init__(self):
        super().__init__('viz_node')
        self.robot = Robot()
        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, 0)
        self.hits_sub = self.create_subscription(Float64MultiArray, 'hits', self.hits_callback, 0)

    def pose_callback(self, msg):
        self.robot.x = msg.position.x
        self.robot.y = msg.position.y
        self.robot.angle = msg.orientation.z

    def hits_callback(self, msg):
        self.robot.hits = np.array(msg.data)


def main(args=None):
    rclpy.init(args=args)
    viz_node = VizNode()

    mapp = Map()

    renderer = Renderer("Vizualisation")
    renderer.set_map(mapp)
    renderer.set_robot(viz_node.robot)

    def update(dt):
        rclpy.spin_once(viz_node, timeout_sec=0)

    pyglet.clock.schedule_interval(update, 1 / 60.0)
    
    pyglet.app.run()
    viz_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()