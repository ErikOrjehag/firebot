
import std_msgs.msg
import rclpy.executors
import rclpy.node
import RPi.GPIO as GPIO
import VL53L1X
import time
import serial

def poll_and_publish_hits(node, executor):
    arduino = serial.Serial('/dev/ttyUSB0', 2000000, timeout=0.005)
    time.sleep(3)
    pub = node.create_publisher(std_msgs.msg.Float64MultiArray, "/hits", 10)
    all = [17, 27, 22, 10, 9, 11, 5, 6]
    xshut = [17, 27, 22, 10, 9, 11, 5]#, 6]
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pin in all:
        GPIO.setup(pin, GPIO.OUT)
    time.sleep(0.1)
    for pin in all:
        GPIO.output(pin, GPIO.LOW)
    for offset, pin in enumerate(xshut, 1):
        GPIO.output(pin, GPIO.HIGH)
        new_address = 0x29 + offset
        node.get_logger().info(new_address)
        arduino.write(new_address)
        time.sleep(1)
    node.get_logger().info("READY")
    while True:
        time.sleep(1)
    tofs = []
    for offset, pin in enumerate(xshut, 1):
        new_address = 0x29 + offset
        node.get_logger().info(f"xshut: {pin}")
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(5)
        tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        node.get_logger().info(f"open: {new_address}")
        tof.open()
        node.get_logger().info(f"set new address: {hex(new_address)}")
        tof.change_address(new_address)
        node.get_logger().info(f"start ranging: {hex(new_address)}")
        tof.set_timing(33000, 38)
        tof.set_user_roi(VL53L1X.VL53L1xUserRoi(7, 8, 7, 8))
        node.get_logger().info(f"start ranging: {hex(new_address)}")
        tof.start_ranging(0)
        tofs.append(tof)
    msg = std_msgs.msg.Float64MultiArray()
    try:
        while rclpy.ok():
            msg.data = [float(tof.get_distance()) / 1000.0 for tof in tofs]
            msg.data.reverse()
            pub.publish(msg)
            executor.spin_once(timeout_sec=0.001)
    except KeyboardInterrupt:
        pass
    for tof in tofs:
        tof.stop_ranging()
        tof.close()


def main():
    rclpy.init()
    node = rclpy.create_node("vl53l1x_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    poll_and_publish_hits(node, executor)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
