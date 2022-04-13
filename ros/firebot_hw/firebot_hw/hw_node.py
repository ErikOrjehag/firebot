
import std_msgs.msg
import rclpy.executors
import rclpy.node
import time
import serial
import collections
import time
import geometry_msgs.msg
import math

WHEEL_BASELINE = 0.17
WHEEL_RADIUS = 0.1 / 2
WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2.0 * math.pi

BASELINE_CIRCUMFERENCE = WHEEL_BASELINE * 2.0 * math.pi
WHEEL_BASELINE_RATIO = BASELINE_CIRCUMFERENCE / WHEEL_CIRCUMFERENCE

N_SENSORS = 8
N_TEMPS = 8
REC_BUF_SIZE = 3 + N_SENSORS * 2 + N_TEMPS
START = b'\x02'
STOP = b'\x03'

sensors = [0.0] * N_SENSORS
temps = [0] * N_TEMPS

def robot_vel_to_wheel_vel(x, z, is_left):
    turn_dir = -1.0 if is_left else 1.0
    angular = z / (2.0*math.pi) * WHEEL_BASELINE_RATIO
    linear = x / WHEEL_CIRCUMFERENCE
    return (linear + turn_dir * angular) * 2.0 * math.pi

def checksum(bts, chs):
    checksum = 0
    for b in bts:
        checksum ^= int.from_bytes(b, 'little')
    return checksum == chs

def run_loop(node, executor):
    arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.005)
    time.sleep(3)
    def cmd_vel_cb(msg: geometry_msgs.msg.Twist):
        s0 = robot_vel_to_wheel_vel(msg.linear.x, -msg.angular.z, False)
        s1 = robot_vel_to_wheel_vel(msg.linear.x, -msg.angular.z, True)
        s = [s0, s1]
        s = [int(w * 1000. / (2. * math.pi)) for w in s]
        s0h = (s[0] >> 8) & 0xFF
        s0l = s[0] & 0xFF
        s1h = (s[1] >> 8) & 0xFF
        s1l = s[1] & 0xFF
        arduino.write(START)
        arduino.write(s0h)
        arduino.write(s0l)
        arduino.write(s1h)
        arduino.write(s1l)
        arduino.write(s0h ^ s0l ^ s1h ^ s1l)
        arduino.write(STOP)

    hits_pub = node.create_publisher(std_msgs.msg.Float64MultiArray, "/hits", 10)
    heat_pub = node.create_publisher(std_msgs.msg.Float64MultiArray, "/heat", 10)
    node.create_subscription(geometry_msgs.msg.Twist, "/cmd_vel", cmd_vel_cb, 10)
    rec_buf = collections.deque([0]*REC_BUF_SIZE, maxlen=REC_BUF_SIZE)
    try:
        n = 0
        ts = time.time()
        while rclpy.ok():
            rec_buf.append(arduino.read())
            if rec_buf[0] == START and rec_buf[-1] == STOP and checksum(list(rec_buf)[1:-2], int.from_bytes(rec_buf[-2], 'little')):
                for i in range(0, N_SENSORS):
                    sensors[i] = int.from_bytes(rec_buf[1+i*2+0], byteorder='little') << 8 | int.from_bytes(rec_buf[1+i*2+1], byteorder='little')
                for i in range(0, N_TEMPS):
                    temps[i] = int.from_bytes(rec_buf[1+N_SENSORS*2+i], byteorder='little')
                n += 1
                if n % 100 == 0:
                    print(f"{100 / (time.time() - ts):.1f} Hz", flush=True)
                    ts = time.time()
                hits_pub.publish(std_msgs.msg.Float64MultiArray(data=sensors))
                heat_pub.publish(std_msgs.msg.Float64MultiArray(data=temps))
            executor.spin_once(timeout_sec=0.001)
    except KeyboardInterrupt:
        pass

def main():
    rclpy.init()
    node = rclpy.create_node("sensor_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    run_loop(node, executor)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()