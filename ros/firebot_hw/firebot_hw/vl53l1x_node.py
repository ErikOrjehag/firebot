
import std_msgs.msg
import rclpy.executors
import rclpy.node
import RPi.GPIO as GPIO
import VL53L1X

def poll_and_publish_sensors(node, executor):
    pub = node.create_publisher(std_msgs.msg.Float64MultiArray, "/hits", 10)
    xshut = [17, 27, 22, 10, 9, 11, 5, 6]
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pin in xshut:
        GPIO.setup(pin, GPIO.OUT)
    for pin in xshut:
        GPIO.output(pin, GPIO.LOW)
    for offset, pin in enumerate(xshut, 1):
        GPIO.output(pin, GPIO.HIGH)
        tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        tof.open()
        tof.change_address(0x29 + offset)
        tof.close()
    tofs = []
    for offset, pin in enumerate(xshut, 1):
        tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29 + offset)
        tof.open()
        tof.set_timing(33000, 38)
        tof.start_ranging(0)
        tofs.append(tof)
    msg = std_msgs.msg.Float64MultiArray()
    try:
        while rclpy.ok():
            msg.data = [float(tof.get_distance()) / 1000.0 for tof in tofs]
            pub.publish(msg)
            executor.spin_once(timeout_sec=0.001)
    except KeyboardInterrupt:
        pass
    for tof in tofs:
        tof.stop_ranging()


def main():
    rclpy.init()
    node = rclpy.create_node("vl53l1x_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    poll_and_publish_sensors(node, executor)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
