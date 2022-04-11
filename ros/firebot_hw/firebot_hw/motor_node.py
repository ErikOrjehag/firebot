
from tkinter import S
import rclpy.executors
import rclpy.node
from smbus2 import SMBus
import geometry_msgs.msg
import math

# Meassure
WHEEL_BASELINE = 0.17
WHEEL_RADIUS = 0.1 / 2
WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2.0 * math.pi

BASELINE_CIRCUMFERENCE = WHEEL_BASELINE * 2.0 * math.pi
WHEEL_BASELINE_RATIO = BASELINE_CIRCUMFERENCE / WHEEL_CIRCUMFERENCE

def robot_vel_to_wheel_vel(x, z, is_left):
    turn_dir = -1.0 if is_left else 1.0
    angular = z / (2.0*math.pi) * WHEEL_BASELINE_RATIO
    linear = x / WHEEL_CIRCUMFERENCE
    return (linear + turn_dir * angular) * 2.0 * math.pi

def motor_node(node, executor):
    
    addr = 0x8
    bus = SMBus(1)

    def sub_callback(msg):

        s0 = robot_vel_to_wheel_vel(msg.linear.x, -msg.angular.z, False)
        s1 = robot_vel_to_wheel_vel(msg.linear.x, -msg.angular.z, True)
        
        s = [s0, s1]
        s = [int(w * 1000. / (2. * math.pi)) for w in s]
        
        s0h = (s[0] >> 8) & 0xFF
        s0l = s[0] & 0xFF
        s1h = (s[1] >> 8) & 0xFF
        s1l = s[1] & 0xFF

        bus.write_byte(addr, 0x10101010)
        bus.write_byte(addr, s0h)
        bus.write_byte(addr, s0l)
        bus.write_byte(addr, s1h)
        bus.write_byte(addr, s1l)
        bus.write_byte(addr, s0h ^ s0l ^ s1h ^ s1l)
        bus.write_byte(addr, 0x01010101)
        node.get_logger().info(f"sent: {s}")

    node.create_subscription(geometry_msgs.msg.Twist, "/cmd_vel", sub_callback, 10)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    bus.close()

def main():
    rclpy.init()
    node = rclpy.create_node("motor_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    while True:
        try:
            motor_node(node, executor)
        except KeyboardInterrupt:
            break
        except Exception:
            pass
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
