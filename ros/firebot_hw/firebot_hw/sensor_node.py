
import std_msgs.msg
import rclpy.executors
import rclpy.node
import time
import serial
import collections
import struct
import time

N_SENSORS = 8
N_TEMPS = 8
REC_BUF_SIZE = 3 + N_SENSORS * 2 + N_TEMPS
START = b'\x02'
STOP = b'\x03'

sensors = [0.0] * N_SENSORS
temps = [0] * N_TEMPS

def checksum(bts, chs):
    checksum = 0
    for b in bts:
        checksum ^= int.from_bytes(b, 'little')
    return checksum == chs

def poll_and_publish_hits(node, executor):
    arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.005)
    time.sleep(3)
    hits_pub = node.create_publisher(std_msgs.msg.Float64MultiArray, "/hits", 10)
    heat_pub = node.create_publisher(std_msgs.msg.Float64MultiArray, "/heat", 10)
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
            executor.spin_once(timeout_sec=0.001)
    except KeyboardInterrupt:
        pass

def main():
    rclpy.init()
    node = rclpy.create_node("sensor_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    poll_and_publish_hits(node, executor)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
