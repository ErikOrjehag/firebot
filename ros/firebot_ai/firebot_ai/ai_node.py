
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseArray
from std_msgs.msg import Float64MultiArray
from firebot_common.robot import Robot
from firebot_common.mapp import Map
from firebot_common.montecarlo import ParticleFilter
import numpy as np
from time import time

class AiNode(Node):

    def __init__(self):
        super().__init__('viz_node')
        self.linear = 0
        self.angular = 0
        self.robot = Robot()
        self.map = Map()
        self.pf = ParticleFilter([
            (50, 0.1, 2.3, 0.1, 2.3),
            #(50, 0.1, 0.8, 0.1, 1.1),
        ])
        self.twist_sub = self.create_subscription(Twist, 'mouse_vel', self.twist_callback, 0)
        self.hits_sub = self.create_subscription(Float64MultiArray, 'hits', self.hits_callback, 0)
        self.pose_pub = self.create_publisher(Pose, 'pose', 0)
        self.pf_pub = self.create_publisher(PoseArray, 'pf', 0)
        self.dt = 0.050
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def hits_callback(self, msg):
        self.robot.hits = np.array(msg.data)
    
    def twist_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z

    def timer_callback(self):
        t1 = time()
        self.pf.update(
            self.dt * self.linear,
            self.dt * self.angular,
            self.robot,
            self.map.walls
        )
        t = time() - t1
        print(f't: {t:.3f}')
        if self.pf.best_pos is not None:
            pose_msg = Pose()
            pose_msg.position.x = self.pf.best_pos[0]
            pose_msg.position.y = self.pf.best_pos[1]
            pose_msg.orientation.z = self.pf.best_angle
            self.pose_pub.publish(pose_msg)
        pf_msg = PoseArray()
        for pos, angle in zip(self.pf.pos, self.pf.angle):
            pose_msg = Pose()
            pose_msg.position.x = pos[0]
            pose_msg.position.y = pos[1]
            pose_msg.orientation.z = angle
            pf_msg.poses.append(pose_msg)
        self.pf_pub.publish(pf_msg)




def main(args=None):
    rclpy.init(args=args)
    ai_node = AiNode()

    rclpy.spin(ai_node)

    ai_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()