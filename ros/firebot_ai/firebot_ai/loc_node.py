
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseArray
from std_msgs.msg import Float64MultiArray, Bool, Float64
from firebot_common.robot import Robot
from firebot_common.mapp import Map
from firebot_common.montecarlo import ParticleFilter
import numpy as np
from time import time
from firebot_common.constants import MAP_SIZE, BODY_RADIUS
import rclpy.qos
from firebot_common.data import rooms

class AiNode(Node):

    def __init__(self):
        super().__init__('viz_node')
        self.linear = 0.0
        self.angular = 0.0
        self.ts = time()
        self.robot = Robot()
        self.map = Map()
        initial_guess = []
        counts = (12, 10, 12, 16)
        for room, count in zip(rooms, counts):
            b = np.array(room["bounds"]) / 100.0
            initial_guess.append([count, b[0] + BODY_RADIUS, b[1] - BODY_RADIUS, b[2] + BODY_RADIUS, b[3] - BODY_RADIUS])
        self.pf = ParticleFilter(initial_guess)
        #([
            #(50, BODY_RADIUS, MAP_SIZE-BODY_RADIUS, BODY_RADIUS, MAP_SIZE-BODY_RADIUS),
            #(50, 0.1, 0.8, 0.1, 1.1),
        #])
        self.twist_sub = self.create_subscription(Twist, "cmd_vel", self.twist_callback, 1)
        self.hits_sub = self.create_subscription(Float64MultiArray, "hits", self.hits_callback, rclpy.qos.qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(Pose, "pose", 1)
        self.pf_pub = self.create_publisher(PoseArray, "pf", 1)
        self.confidence_pub = self.create_publisher(Float64, "confidence", 1)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def hits_callback(self, msg):
        self.robot.hits = np.array(msg.data)
    
    def twist_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z
        t = time()
        dt = t - self.ts
        if dt > 0.2:
            dt = 0.2
        self.ts = t
        linear = dt * msg.linear.x
        angular = dt * msg.angular.z
        self.pf.angle += angular
        self.pf.pos += linear*np.vstack((np.cos(self.pf.angle), np.sin(self.pf.angle))).T

    def timer_callback(self):
        # Localization
        t1 = time()
        for _ in range(1):
            self.pf.update(
                self.linear,
                self.angular,
                self.robot,
                self.map.walls
            )
        t = time() - t1
        # self.get_logger().info(f't: {t:.3f}, confidence: {self.pf.confidence:.3f}')
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
        self.confidence_pub.publish(Float64(data=self.pf.confidence))

def main(args=None):
    rclpy.init(args=args)
    ai_node = AiNode()

    rclpy.spin(ai_node)

    ai_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
