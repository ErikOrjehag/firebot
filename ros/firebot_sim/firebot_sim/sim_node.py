
import pyglet
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from firebot_sim.simulation import Simulation
from firebot_common.renderer import Renderer

class SimNode(Node):

    def __init__(self):
        super().__init__('viz_node')
        self.simulation = Simulation()
        self.pose_pub = self.create_publisher(Pose, 'pose', 0)
        self.hits_pub = self.create_publisher(Float64MultiArray, 'hits', 0)
        self.pub_timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        pose_msg = Pose()
        pose_msg.position.x = self.simulation.robot.x
        pose_msg.position.y = self.simulation.robot.y
        pose_msg.orientation.z = self.simulation.robot.angle
        self.pose_pub.publish(pose_msg)
        hits_msg = Float64MultiArray()
        for hit in self.simulation.robot.hits:
            hits_msg.data.append(hit)
        self.hits_pub.publish(hits_msg)

def main(args=None):
    rclpy.init(args=args)
    sim_node = SimNode()
    
    def update(dt):
        sim_node.simulation.update(dt)
        rclpy.spin_once(sim_node, timeout_sec=0)
    
    pyglet.clock.schedule_interval(update, 1 / 60.0)
    
    pyglet.app.run()
    sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()