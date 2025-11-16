import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import freqz

# ================================
# Parameters
# ================================
fs = 48000       # sample rate for normalization only

# ================================
# IIR DC-blocker (leaky integrator)
# H(z) = (1 - z^-1) / (1 - p z^-1)
# Difference eq: y[n] = p*y[n-1] + x[n] - x[n-1]
# ================================
b_iir = np.array([1.0, -1.0])   # numerator
a_iir995 = np.array([1.0, -0.995])     # denominator

w_iir995, h_iir995 = freqz(b_iir, a_iir995, worN=4096)

a_iir9993 = np.array([1.0, -0.9993])     # denominator
w_iir9993, h_iir9993 = freqz(b_iir, a_iir9993, worN=4096)

# =================================
# IIR Butterworth high-pass filter
# 2nd order, fc = 10 Hz, fs = 48 kHz
# =================================
from scipy.signal import butter, sosfreqz
sos = butter(2, 10, btype='highpass', fs=fs, output='sos')
w_butt, h_butt = sosfreqz(sos, worN=4096, fs=fs)
print(sos)

# ================================
# FIR DC-blocker (moving average subtraction)
# y[n] = x[n] - (1/D)*sum_{k=0}^{D-1} x[n-k]
# i.e. an FIR filter: h = [1, 0, 0, ..., 0] - (1/D)*[1,1,...,1]
# ================================
def get_fir_ma_dc_blocker(D):
  fir_ma = np.ones(D) / D
  b_fir = np.zeros(D)
  b_fir[0] = 1.0
  b_fir -= fir_ma
  return b_fir
b_fir32 = get_fir_ma_dc_blocker(32)
b_fir64 = get_fir_ma_dc_blocker(64)

w_fir, h_fir = freqz(b_fir32, [1.0], worN=4096)
w_fir64, h_fir64 = freqz(b_fir64, [1.0], worN=4096)

# ===============================
# FIR DC-blocker (differentiator)
# y[n] = x[n] - x[n-1]
# i.e. an FIR filter: h = [1, -1]
# ===============================
b_fir_diff = np.array([1.0, -1.0])
w_fir_diff, h_fir_diff = freqz(b_fir_diff, [1.0], worN=4096)


# ================================
# Plot magnitude response
# ================================
plt.figure(figsize=(10, 6))
plt.title("DC-blocker Frequency Responses")
plt.plot(w_iir995/np.pi*fs/2, 20*np.log10(np.abs(h_iir995)+1e-12),
         label=f"IIR DC blocker (p={0.995})")
plt.plot(w_iir9993/np.pi*fs/2, 20*np.log10(np.abs(h_iir9993)+1e-12),
         label=f"IIR DC blocker (p={0.9993})")
plt.plot(w_fir/np.pi*fs/2, 20*np.log10(np.abs(h_fir)+1e-12),
         label=f"FIR MA DC blocker (D={32})")
plt.plot(w_fir64/np.pi*fs/2, 20*np.log10(np.abs(h_fir64)+1e-12),
         label=f"FIR MA DC blocker (D={64})")
plt.plot(w_fir_diff/np.pi*fs/2, 20*np.log10(np.abs(h_fir_diff)+1e-12),
         label="FIR diff DC blocker")
plt.plot(w_butt, 20*np.log10(np.abs(h_butt)+1e-12),
         label="IIR Butterworth high-pass")

plt.xscale("log")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude (dB)")
plt.ylim([-80, 10])
plt.grid(True, which="both")
plt.legend()

plt.show()


# ================================
# Plot phase response
# ================================
plt.figure(figsize=(10, 6))
plt.title("Phase Responses")
plt.plot(w_iir995/np.pi*fs/2, np.unwrap(np.angle(h_iir995)),
         label="IIR DC blocker (p=0.995)")
plt.plot(w_iir9993/np.pi*fs/2, np.unwrap(np.angle(h_iir9993)),
         label="IIR DC blocker (p=0.9993)")
plt.plot(w_fir/np.pi*fs/2, np.unwrap(np.angle(h_fir)),
         label="FIR MA DC blocker (D=32)")
plt.plot(w_fir64/np.pi*fs/2, np.unwrap(np.angle(h_fir64)),
         label="FIR MA DC blocker (D=64)")
plt.plot(w_fir_diff/np.pi*fs/2, np.unwrap(np.angle(h_fir_diff)),
         label="FIR diff DC blocker")
plt.plot(w_butt, np.unwrap(np.angle(h_butt)),
         label="IIR Butterworth high-pass")

plt.xscale("log")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Phase (radians)")
plt.grid(True, which="both")
plt.legend()
plt.tight_layout()
plt.show()
