
import std_msgs.msg
import rclpy.executors
import rclpy.node
import RPi.GPIO as GPIO

def control_servo(node, executor):
    SERVO_PIN = 13
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    p = GPIO.PWM(SERVO_PIN, 50)
    UP_DUTY = 2.5
    DOWN_DUTY = 5.0
    p.start(UP_DUTY)

    def sub_callback(msg):
        p.ChangeDutyCycle(DOWN_DUTY if msg.data else UP_DUTY)

    sub = node.create_subscription(std_msgs.msg.Bool, "/snuff", sub_callback, 10)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    p.stop()
    GPIO.cleanup()

def main():
    rclpy.init()
    node = rclpy.create_node("servo_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    control_servo(node, executor)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
