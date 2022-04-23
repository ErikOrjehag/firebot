
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseArray
from std_msgs.msg import Float64MultiArray, Bool, Float64, Empty
from firebot_common.robot import Robot
from firebot_common.mapp import Map
from firebot_common.montecarlo import ParticleFilter
import numpy as np
from time import time
from firebot_common.constants import MAP_SIZE, BODY_RADIUS, N_SEARCH_CELLS, N_SEARCH_CELLS
import rclpy.qos
from firebot_common.data import rooms

class AiNode(Node):

    def __init__(self):
        super().__init__('viz_node')
        self.linear = 0.0
        self.angular = 0.0
        self.start = False
        self.hw_ready = False
        self.ts = time()
        self.robot = Robot()
        self.map = Map()
        initial_guess = []
        counts = (12, 10, 12, 16)
        for room, count in zip(rooms, counts):
            b = np.array(room["bounds"]) / 100.0
            initial_guess.append([count, b[0] + BODY_RADIUS, b[1] - BODY_RADIUS, b[2] + BODY_RADIUS, b[3] - BODY_RADIUS])
        self.pf = ParticleFilter(initial_guess)
        
        self.create_subscription(Twist, "cmd_vel", self.twist_callback, 1)
        self.create_subscription(Float64MultiArray, "hits", self.hits_callback, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Empty, "start", self.start_callback, 1)
        
        self.create_subscription(Float64MultiArray, 'dijkstras', self.dijkstras_callback, 5)

        self.led_green_pub = self.create_publisher(Bool, "led/green", 1)
        self.led_yellow_pub = self.create_publisher(Bool, "led/yellow", 1)
        self.led_red_pub = self.create_publisher(Bool, "led/red", 1)
        
        self.pose_pub = self.create_publisher(Pose, "pose", 1)
        self.pf_pub = self.create_publisher(PoseArray, "pf", 1)
        self.confidence_pub = self.create_publisher(Float64, "confidence", 1)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def dijkstras_callback(self, msg):
        self.get_logger().info("got dijkstas")
        self.pf.dijkstras = np.array(msg.data).reshape((N_SEARCH_CELLS, N_SEARCH_CELLS))

    def hits_callback(self, msg):
        self.robot.hits = np.array(msg.data)
        self.hw_ready = True
    
    def start_callback(self, msg):
        self.start = True
        self.led_green_pub.publish(Bool(data=True))
        self.led_yellow_pub.publish(Bool(data=False))

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
        if not self.start:
            self.led_red_pub.publish(Bool(data=False))
            self.led_green_pub.publish(Bool(data=False))
            self.led_yellow_pub.publish(Bool(data=int(time()) % 2 == 0))
            self.get_logger().info("Waiting for start signal", throttle_duration_sec=3.0)
            return

        if not self.hw_ready:
            self.get_logger().info("Waiting for hw ready", throttle_duration_sec=3.0)
            return

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
