import numpy as np
from utils import ms2smp, compute_stride, win_taper, build_linear_interp_table
import sounddevice as sd
import time as timee

"""
Real-time pitch shifting with granular synthesis for shift factors <=1.0
"""

""" User selected parameters """
grain_len = 30
grain_over = 0.3
shift_factor = 0.6 
data_type = np.int16
samp_freq = 16000

# derived parameters
MAX_VAL = np.iinfo(data_type).max
GRAIN_LEN_SAMP = ms2smp(grain_len, samp_freq)
STRIDE = compute_stride(GRAIN_LEN_SAMP, grain_over)
OVERLAP_LEN = GRAIN_LEN_SAMP-STRIDE

# allocate input and output buffers
input_buffer = np.zeros(STRIDE, dtype=data_type)
output_buffer = np.zeros(STRIDE, dtype=data_type)


# state variables and constants
def init():
    global WIN
    WIN = win_taper(GRAIN_LEN_SAMP, grain_over, data_type)
    WIN = np.array(WIN, dtype=np.int32)

    global SAMP_VALS, AMP_VALS, SAMP_VALS_NEXT, AMP_VALS_NEXT
    SAMP_VALS, AMP_VALS = build_linear_interp_table(GRAIN_LEN_SAMP, shift_factor, data_type)
    AMP_VALS_NEXT = MAX_VAL - AMP_VALS
    AMP_VALS = np.array(AMP_VALS, dtype=np.int32)  
    AMP_VALS_NEXT = np.array(AMP_VALS_NEXT, dtype=np.int32)
    SAMP_VALS_NEXT = np.array(np.add(SAMP_VALS, 1), dtype=data_type)

    global GRAIN_PREV, GRAIN_CURRENT, GRAIN_INTERP
    GRAIN_PREV = np.zeros(OVERLAP_LEN, dtype=np.int64)
    GRAIN_CURRENT = np.zeros(GRAIN_LEN_SAMP, dtype=np.int64) 
    GRAIN_INTERP = np.zeros(GRAIN_LEN_SAMP, dtype=np.int64)

    global GRAIN_TEMP
    GRAIN_TEMP = np.zeros(GRAIN_LEN_SAMP, dtype=np.int32)

    global X_PREV, X_CONCAT
    X_PREV = np.zeros(OVERLAP_LEN, dtype=data_type)
    X_CONCAT = np.empty(GRAIN_LEN_SAMP, dtype=np.int32)

# the process function!
def process(input_buffer, output_buffer, buffer_len):
    input_buffer = (input_buffer.astype(np.int32) * 1) // 10
    input_buffer = input_buffer.astype(data_type)

    X_CONCAT[:OVERLAP_LEN] = X_PREV
    X_CONCAT[OVERLAP_LEN:] = input_buffer

    np.take(X_CONCAT, SAMP_VALS, out=GRAIN_TEMP)
    np.copyto(GRAIN_CURRENT, GRAIN_TEMP)
    np.take(X_CONCAT, SAMP_VALS_NEXT, out=GRAIN_TEMP)
    np.copyto(GRAIN_INTERP, GRAIN_TEMP)
    np.multiply(GRAIN_CURRENT, AMP_VALS, out=GRAIN_CURRENT)
    np.multiply(GRAIN_INTERP, AMP_VALS_NEXT, out=GRAIN_INTERP)
    np.add(GRAIN_CURRENT, GRAIN_INTERP, out=GRAIN_CURRENT)
    np.multiply(GRAIN_CURRENT, WIN, out=GRAIN_CURRENT)
    np.right_shift(GRAIN_CURRENT, 30, out=GRAIN_CURRENT)
    np.add(GRAIN_CURRENT[:OVERLAP_LEN], GRAIN_PREV, out=GRAIN_CURRENT[:OVERLAP_LEN])

    GRAIN_PREV[:] = GRAIN_CURRENT[STRIDE:]
    X_PREV[:] = X_CONCAT[-OVERLAP_LEN:]
    np.multiply(GRAIN_CURRENT, 10, out=GRAIN_CURRENT)
    np.clip(GRAIN_CURRENT, -MAX_VAL, MAX_VAL, out=GRAIN_CURRENT)
    output_buffer[:] = GRAIN_CURRENT[:STRIDE].astype(data_type)
    


"""
# Nothing to touch after this!
# """
try:
    sd.default.samplerate = samp_freq
    sd.default.blocksize = STRIDE
    sd.default.dtype = data_type

    def callback(indata, outdata, frames, time, status):
        if np.max(np.abs(indata)) >= 32767:
            print("WARNING: Input signal is clipping!")

        process(indata[:,0], outdata[:,0], frames)

    init()
    with sd.Stream(channels=1, callback=callback):
        print('#' * 80)
        print('press Return to quit')
        print('#' * 80)
        input()
except KeyboardInterrupt:
    print('\nInterrupted by user')




