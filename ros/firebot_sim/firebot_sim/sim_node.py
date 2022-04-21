
import pyglet
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from firebot_sim.simulation import Simulation

class SimNode(Node):

    def __init__(self):
        super().__init__('viz_node')
        self.simulation = Simulation()
        self.hits_pub = self.create_publisher(Float64MultiArray, 'hits', 0)
        self.twist_sub = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 0)
        self.pub_timer = self.create_timer(0.02, self.timer_callback)

    def twist_callback(self, msg):
        self.simulation.linear = msg.linear.x
        self.simulation.angular = msg.angular.z

    def timer_callback(self):
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