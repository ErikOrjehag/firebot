

import pyaudio
import numpy as np
import scipy.signal
import struct
import math

# p = pyaudio.PyAudio()
# for ii in range(p.get_device_count()):
#     print(f"{ii}) {p.get_device_info_by_index(ii).get('name')}")

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

def block_until_start_signal():
    stream = pyaudio.PyAudio().open(format=pyaudio.paInt16,
                                channels=1,
                                rate=RATE,
                                input=True,
                                frames_per_buffer=INPUT_FRAMES_PER_BLOCK,
                                input_device_index=0)

    b, a = design_filter(950.0, 1050.0, RATE, 3)
    zi = scipy.signal.lfilter_zi(b, a)

    stream.start_stream()

    while stream.is_active():
        block = stream.read(INPUT_FRAMES_PER_BLOCK, exception_on_overflow=False)
        samples = normalize(block)
        bandpass_samples, zi = scipy.signal.lfilter(b, a, samples, zi=zi)
        bandpass_ampl = get_rms(bandpass_samples)
        print(f"Amplitude: {bandpass_ampl:.3f}")
        if bandpass_ampl > 0.1:
            break