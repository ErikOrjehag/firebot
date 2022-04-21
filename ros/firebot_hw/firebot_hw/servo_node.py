
import std_msgs.msg
import rclpy.executors
import rclpy.node
import RPi.GPIO as GPIO

SERVO_PIN = 13
LED_GREEN_PIN = 16
LED_RED_PIN = 25
LED_YELLOW_PIN = 12
BUZZ_PIN = 21
BTN_GREEN_PIN = 24
BTN_RED_PIN = 23

SERVO_UP_DUTY = 2.5
SERVO_DOWN_DUTY = 5.0
    
def control_servo(node, executor):
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    p = GPIO.PWM(SERVO_PIN, 50)
    p.start(SERVO_UP_DUTY)

    def snuff_cb(msg: std_msgs.msg.Bool):
        p.ChangeDutyCycle(SERVO_DOWN_DUTY if msg.data else SERVO_UP_DUTY)

    node.create_subscription(std_msgs.msg.Bool, "/snuff", snuff_cb, 10)

    GPIO.setup(LED_GREEN_PIN, GPIO.OUT)
    GPIO.setup(LED_YELLOW_PIN, GPIO.OUT)
    GPIO.setup(LED_RED_PIN, GPIO.OUT)
    GPIO.setup(BUZZ_PIN, GPIO.OUT)
    GPIO.setup(BTN_RED_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN_GREEN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def led_green_cb(msg: std_msgs.msg.Bool):
        GPIO.output(LED_GREEN_PIN, GPIO.HIGH if msg.data else GPIO.LOW)

    def led_yellow_cb(msg: std_msgs.msg.Bool):
        GPIO.output(LED_YELLOW_PIN, GPIO.HIGH if msg.data else GPIO.LOW)

    def led_red_cb(msg: std_msgs.msg.Bool):
        GPIO.output(LED_RED_PIN, GPIO.HIGH if msg.data else GPIO.LOW)

    def led_buzz_cb(msg: std_msgs.msg.Bool):
        GPIO.output(BUZZ_PIN, GPIO.HIGH if msg.data else GPIO.LOW)

    node.create_subscription(std_msgs.msg.Bool, '/led/green', led_green_cb, 10)
    node.create_subscription(std_msgs.msg.Bool, '/led/yellow', led_yellow_cb, 10)
    node.create_subscription(std_msgs.msg.Bool, '/led/red', led_red_cb, 10)
    node.create_subscription(std_msgs.msg.Bool, '/buzz', led_buzz_cb, 10)

    btn_green_pub = node.create_publisher(std_msgs.msg.Bool, "/btn/green", 10)
    btn_red_pub = node.create_publisher(std_msgs.msg.Bool, "/btn/red", 10)

    def btn_timer_cb():
        btn_green_pub.publish(std_msgs.msg.Bool(data=(GPIO.input(BTN_GREEN_PIN) == GPIO.LOW)))
        btn_red_pub.publish(std_msgs.msg.Bool(data=(GPIO.input(BTN_RED_PIN) == GPIO.LOW)))

    node.create_timer(0.1, btn_timer_cb)

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
