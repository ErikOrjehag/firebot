
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseArray
from std_msgs.msg import Float64MultiArray, Bool, Float64
from firebot_common.mapp import Map
import numpy as np
from time import time
from firebot_common.constants import MAP_SIZE, BODY_RADIUS, HEAT_FOV
import rclpy.qos

def signed_limit(value, limit) -> float:
    return np.sign(value) * min(abs(value), limit)

def safe_sleep(node, t):
    ts = time()
    while time() - ts < t:
        rclpy.spin_once(node, timeout_sec=0.01)

class AiNode(Node):

    def __init__(self):
        super().__init__('viz_node')
        self.heat = [0] * 8
        self.pose = None
        
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.snuff_pub = self.create_publisher(Bool, "snuff", 1)

        self.create_subscription(Float64MultiArray, "hits", self.hits_callback, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, "heat", self.heat_callback, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Float64, "confidence", self.confidence_callback, 1)
        self.create_subscription(Pose, "pose", self.pose_callback, 1)

    def confidence_callback(self, msg: Float64):
        self.confidence = msg.data

    def heat_callback(self, msg: Float64MultiArray):
        self.heat = msg.data

    def hits_callback(self, msg: Float64MultiArray):
        self.hits = np.array(msg.data)
    
    def pose_callback(self, msg: Pose):
        self.pos = np.array([msg.position.x, msg.position.y])
        self.angle = msg.orientation.z

    def run(self):
        
        SNUFF_DIST = 0.04
        ts = time()

        while rclpy.ok():
            t = time()
            dt = t - ts
            ts = t

            # Heat sensor
            alphas = np.linspace(HEAT_FOV/2, -HEAT_FOV/2, 8)
            if np.max(self.heat) > 50.0:
                alpha = np.average(alphas, weights=self.heat)
                msg = Twist()
                msg.angular.z = signed_limit(dt * 20.0 * alpha, 0.1)
                msg.linear.x = signed_limit(0.1 * (self.hits[0] - SNUFF_DIST), 0.05)
                self.cmd_vel_pub.publish(msg)

                if abs(self.hits[0] - SNUFF_DIST) < 0.01:
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

            rclpy.spin_once(self, timeout_sec=0.01)
        

def main(args=None):
    rclpy.init(args=args)
    ai_node = AiNode()

    try:
        ai_node.run()
    except KeyboardInterrupt:
        pass

    ai_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
