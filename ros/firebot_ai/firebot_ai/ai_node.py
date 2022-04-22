
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseArray
from std_msgs.msg import Float64MultiArray, Bool
from firebot_common.robot import Robot
from firebot_common.mapp import Map
from firebot_common.montecarlo import ParticleFilter
import numpy as np
from time import time
from firebot_common.constants import MAP_SIZE, BODY_RADIUS

def signed_limit(value, limit) -> float:
    return np.sign(value) * min(abs(value), limit)

def safe_sleep(node, t):
    ts = time()
    while time() - ts < t:
        rclpy.spin_once(node, timeout_sec=0.01)

class AiNode(Node):

    def __init__(self):
        super().__init__('viz_node')
        self.linear = 0.0
        self.angular = 0.0
        self.ts = time()
        self.robot = Robot()
        self.map = Map()
        self.heat = [0] * 8
        self.pf = ParticleFilter([
            (50, BODY_RADIUS, MAP_SIZE-BODY_RADIUS, BODY_RADIUS, MAP_SIZE-BODY_RADIUS),
            #(50, 0.1, 0.8, 0.1, 1.1),
        ])
        self.twist_sub = self.create_subscription(Twist, "cmd_vel", self.twist_callback, 5)
        self.hits_sub = self.create_subscription(Float64MultiArray, "hits", self.hits_callback, 5)
        self.heat_sub = self.create_subscription(Float64MultiArray, "heat", self.heat_callback, 5)
        self.pose_pub = self.create_publisher(Pose, "pose", 5)
        self.pf_pub = self.create_publisher(PoseArray, "pf", 5)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 5)
        self.snuff_pub = self.create_publisher(Bool, "snuff", 5)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def heat_callback(self, msg: Float64MultiArray):
        self.heat = msg.data

    def hits_callback(self, msg):
        self.robot.hits = np.array(msg.data)
    
    def twist_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z
        t = time()
        dt = t - self.ts
        self.ts = t
        linear = dt * msg.linear.x
        angular = dt * msg.angular.z
        self.pf.angle += angular
        self.pf.pos += linear*np.vstack((np.cos(self.pf.angle), np.sin(self.pf.angle))).T

    def timer_callback(self):
        # Localization
        t1 = time()
        for _ in range(5):
            self.pf.update(
                self.linear,
                self.angular,
                self.robot,
                self.map.walls
            )
        t = time() - t1
        self.get_logger().info(f't: {t:.3f}')
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
        
        SNUFF_DIST = 0.04

        # Heat sensor
        alphas = np.linspace(self.robot.heat_fov/2, -self.robot.heat_fov/2, 8)
        if np.max(self.heat) > 50.0:
            alpha = np.average(alphas, weights=self.heat)
            msg = Twist()
            msg.angular.z = signed_limit(self.dt * 20.0 * alpha, 0.1)
            msg.linear.x = signed_limit(0.1 * (self.robot.hits[0] - SNUFF_DIST), 0.05)
            self.cmd_vel_pub.publish(msg)

            if abs(self.robot.hits[0] - SNUFF_DIST) < 0.01:
                self.get_logger().info("SNUFF!")
                self.snuff_pub.publish(Bool(data=True))
                safe_sleep(self, 3.0)
                self.snuff_pub.publish(Bool(data=False))
                msg = Twist()
                msg.linear.x = -0.05
                ts = time()
                while time() - ts < 3.0:
                    self.cmd_vel_pub.publish(msg)
                self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    ai_node = AiNode()

    rclpy.spin(ai_node)

    ai_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
