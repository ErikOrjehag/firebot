
import std_msgs.msg
import rclpy.executors
import rclpy.node
from firebot_hw.tpa81 import TPA81
import time

def poll_and_publish_heat(node, executor):
    pub = node.create_publisher(std_msgs.msg.Float64MultiArray, "/heat", 10)
    
    tpa = TPA81(bus_num=1)

    node.get_logger().info(f"Version: {tpa.softwareVersion()}")
    node.get_logger().info(f"Ambient temperature: {tpa.ambientTemperature()}")

    msg = std_msgs.msg.Float64MultiArray()
    try:
        while rclpy.ok():
            heat = []
            i = 0
            while i < len(tpa.TPA81_PIXEL):
                time.sleep(0.01)
                t = tpa.pixelTemp(i)
                if t == 0:
                    print("is zero")
                    continue
                heat.append(float(t))
                i = i + 1
            msg.data = heat
            pub.publish(msg)
            executor.spin_once(timeout_sec=0.001)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


def main():
    rclpy.init()
    node = rclpy.create_node("tpa81_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    poll_and_publish_heat(node, executor)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
