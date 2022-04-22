
import std_msgs.msg
import rclpy.executors
import rclpy.node
import pyaudio
import numpy as np
import scipy.signal
import struct
import math
import std_msgs.msg

SHORT_NORMALIZE = 1.0 / 1000.0

RATE = 48000 
INPUT_BLOCK_TIME = 1
INPUT_FRAMES_PER_BLOCK = int(RATE * INPUT_BLOCK_TIME)

def design_filter(lowcut, highcut, fs, order=3):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b,  a = scipy.signal.butter(order, [low, high], btype='band')
    return b, a

def normalize(block):
    count = len(block) / 2
    format = "%dh" % (count)
    shorts = struct.unpack(format, block)
    doubles = np.array([x * SHORT_NORMALIZE for x in shorts])
    return doubles

def get_rms(samples):
    sum_squares = 0.0
    for sample in samples:
        sum_squares += sample * sample
    return math.sqrt(sum_squares / len(samples))

def poll_mic(node, executor):
    node.get_logger().info("Starting mic node")

    start_pub = node.create_publisher(
        std_msgs.msg.Empty, "/start", 1)

    p = pyaudio.PyAudio()

    for device_index in range(p.get_device_count()):
        dev = p.get_device_info_by_index(device_index)
        if "USB PnP Sound Device" in dev["name"]:
            break

    stream = p.open(format=pyaudio.paInt16,
                                channels=1,
                                rate=RATE,
                                input=True,
                                frames_per_buffer=INPUT_FRAMES_PER_BLOCK,
                                input_device_index=device_index)

    b, a = design_filter(950.0, 1050.0, RATE, 3)
    zi = scipy.signal.lfilter_zi(b, a)

    stream.start_stream()

    try:
        while stream.is_active():
            block = stream.read(INPUT_FRAMES_PER_BLOCK, exception_on_overflow=False)
            samples = normalize(block)
            bandpass_samples, zi = scipy.signal.lfilter(b, a, samples, zi=zi)
            bandpass_ampl = get_rms(bandpass_samples)
            # print(f"Amplitude: {bandpass_ampl:.3f}")
            if bandpass_ampl > 0.1:
                start_pub.publish(std_msgs.msg.Empty())
            executor.spin_once(timeout_sec=0.001)
    except KeyboardInterrupt:
        pass

def main():
    rclpy.init()
    node = rclpy.create_node("servo_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    poll_mic(node, executor)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
