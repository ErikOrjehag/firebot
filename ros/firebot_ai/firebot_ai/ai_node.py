
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseArray
from std_msgs.msg import Float64MultiArray, Bool, Float64
from firebot_common.calc_hits import angle_between
from firebot_common.mapp import Map
import numpy as np
from time import time
from firebot_common.constants import MAP_SIZE, BODY_RADIUS, HEAT_FOV, SEARCH_CELL_SIZE
import rclpy.qos
import firebot_common.wall_collision
import firebot_common.dijkstras
from math import cos, sin, pi, sqrt
from firebot_common.data import rooms

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
        self.pos = None
        self.angle = None
        self.confidence = 0.0
        self.home = None
        self.room_plan = None
        self.goal = None
        self.localizing_ts = None
        self.map = Map()
        self.distmap = firebot_common.wall_collision.generate_distance_map(self.map)
        self.dijkstras = None

        self.look_for_candle = False

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.snuff_pub = self.create_publisher(Bool, "snuff", 1)
        self.carrot_pub = self.create_publisher(Pose, "carrot", 1)

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
        
        SNUFF_DIST = 0.03
        MAX_ANGULAR = 0.3
        MAX_LINEAR = 0.05
        ts = time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            t = time()
            dt = t - ts
            ts = t

            if self.pos is None or self.angle is None:
                continue

            if self.home is None:
                self.get_logger().info(f"Looking for home", throttle_duration_sec=5.0)
                if self.confidence > 0.99:
                    for i, room in enumerate(rooms):
                        b = np.array(room["bounds"]) / 100.0
                        xmin, xmax, ymin, ymax = b
                        x, y = self.pos
                        if xmin <= x <= xmax and ymin <= y <= ymax:
                            self.home = i
                            self.room_plan = [
                                [1, 3, 2],
                                [0, 3, 2],
                                [3, 1, 0],
                                [2, 1, 0],
                            ][self.home]
                            self.get_logger().info(f"Setting home: {i:d}, with plan: {self.room_plan}")
                            break

            if self.dijkstras is None and self.room_plan is not None:
                if len(self.room_plan):
                    room_i = self.room_plan[0]
                    self.get_logger().info(f"Going to room: {room_i}")
                    self.room_plan = self.room_plan[1:]
                    b = np.array(rooms[room_i]["bounds"]) / 100.0
                    self.goal = np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2])
                    self.dijkstras = firebot_common.dijkstras.dijkstras_search(self.distmap, self.goal[0], self.goal[1]) + 0.3 * np.clip(1.0 - self.distmap / (BODY_RADIUS*2), 0.0, 1.0)
                else:
                    self.get_logger().info("No rooms left to explore!")

            # Heat sensor
            alphas = np.linspace(HEAT_FOV/2, -HEAT_FOV/2, 8)
            if np.max(self.heat) > 50.0:
                alpha = np.average(alphas, weights=self.heat)
                msg = Twist()
                msg.angular.z = signed_limit(dt * 100.0 * alpha, MAX_ANGULAR)
                if alpha < 5.0 * pi / 180:
                    msg.linear.x = signed_limit(0.1 * (self.hits[0] - SNUFF_DIST), 0.05)
                self.cmd_vel_pub.publish(msg)

                if abs(self.hits[0] - SNUFF_DIST) < 0.01:
                    self.get_logger().info("SNUFF!")
                    self.snuff_pub.publish(Bool(data=True))
                    safe_sleep(self, 4.0)
                    self.snuff_pub.publish(Bool(data=False))
                    msg = Twist()
                    msg.linear.x = -0.05
                    ts = time()
                    while time() - ts < 3.0:
                        self.cmd_vel_pub.publish(msg)
                    self.cmd_vel_pub.publish(Twist())

            else:

                if self.dijkstras is not None and self.confidence > 0.8:
                    self.localizing_ts = None

                    if np.linalg.norm(self.goal - self.pos) < 0.1:
                        self.look_for_candle = True
                    
                    if self.look_for_candle:
                        self.get_logger().info("Look for candle", throttle_duration_sec=3.0)
                        msg = Twist()
                        msg.angular.z = 0.3
                        self.cmd_vel_pub.publish(msg)
                    else:
                        msg = Twist()
                        if self.hits[0] < 0.15:
                            msg.linear.x = -0.05
                        elif self.hits[1] < 0.05:
                            msg.angular.z = -0.4
                        elif self.hits[-1] < 0.05:
                            msg.angular.z = 0.4
                        else:
                            # pt = self.pos.copy()
                            # steplen = sqrt(2)*SEARCH_CELL_SIZE
                            # for _ in range(2):
                            #     flowdir = firebot_common.dijkstras.flow(self.dijkstras, *pt)
                            #     pt += steplen * flowdir
                            # carrot_pose = Pose()
                            # carrot_pose.position.x = pt[0]
                            # carrot_pose.position.y = pt[1]
                            # self.carrot_pub.publish(carrot_pose)
                            # carrot_dir = (pt - self.pos)
                            # carrot_dir /= np.linalg.norm(carrot_dir)
                            carrot_dir = firebot_common.dijkstras.flow(self.dijkstras, *self.pos)
                            robot_dir = np.array([cos(self.angle), sin(self.angle)])
                            alpha = angle_between(carrot_dir, robot_dir)
                            msg.angular.z = signed_limit(dt * 10.0 * alpha, MAX_ANGULAR)
                            #self.get_logger().info(f"{abs(msg.angular.z):.3f} {(MAX_ANGULAR / 2 - abs(msg.angular.z)):.3f} {np.clip((MAX_ANGULAR - abs(msg.angular.z)) / (MAX_ANGULAR), 0, 1):.3f}")
                            msg.linear.x = MAX_LINEAR * np.clip((MAX_ANGULAR/6 - abs(msg.angular.z)) / (MAX_ANGULAR/6), 0.1, 1)
                        self.cmd_vel_pub.publish(msg)
                else:
                    self.get_logger().info("Localizing", throttle_duration_sec=3.0)
                    # if self.localizing_ts is None:
                    #     self.localizing_ts = time()
                    # seqi = int(self.localizing_ts / 3.0) % 4
                    # self.get_logger().info(f"{seqi:d}")
                    # msg = Twist()
                    # if seqi in [0]:
                    #     pass
                    # elif seqi in [1]:
                    #     msg.linear.x = -0.03
                    # elif seqi in [2]:
                    #     msg.angular.z = -0.05
                    # elif seqi in [3]:
                    #     msg.angular.z = 0.05
                    # self.cmd_vel_pub.publish(msg)
        

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
