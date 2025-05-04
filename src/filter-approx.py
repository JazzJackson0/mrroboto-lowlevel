#!/usr/bin/python
import scipy.signal as signal
import csv
import numpy as np
import matplotlib.pyplot as plt

# Low Pass Filter-----------------------------------------------------
# # Filter Specs
# order = 4
# cutoff = 0.3
# fs = 100  # Hz

# b, a = signal.iirfilter(order, cutoff, btype='low', ftype='butter', fs=fs)
# secondOrders = signal.tf2sos(b, a) # Convert coefs to Biquad format

# fo = open("../doc/iir-coeffs.csv", "w", newline="")
# writer = csv.writer(fo)
# writer.writerows(secondOrders)
# fo.close()

# '''
# secondOrders array
# [[b00, b01, b02, a00, a01, a02],
#  [b10, b11, b12, a10, a11, a12]]
# '''

# High Pass Filter-----------------------------------------------------
# Filter Specs
order = 4
cutoff = 5
fs = 100  # Hz

b, a = signal.iirfilter(order, cutoff, btype='high', ftype='butter', fs=fs)
secondOrders = signal.tf2sos(b, a) # Convert coefs to Biquad format

fo = open("../doc/iir-highpass-coeffs.csv", "w", newline="")
writer = csv.writer(fo)
writer.writerows(secondOrders)
fo.close()

'''
secondOrders array
[[b00, b01, b02, a00, a01, a02],
 [b10, b11, b12, a10, a11, a12]]
'''

# FFT Analysis-----------------------------------------------------------
# # Example: Simulated gyroscope data with multiple frequencies
# fs = 100  # Sampling frequency (Hz)
# t = np.arange(0, 2, 1/fs)  # 2 seconds of data
# gyro_data = np.sin(2 * np.pi * 5 * t) + 0.5 * np.sin(2 * np.pi * 20 * t)  # 5Hz + 20Hz components

# # Compute FFT
# N = len(gyro_data)
# freqs = np.fft.fftfreq(N, d=1/fs)[:N//2]  # Only positive frequencies
# fft_magnitude = np.abs(np.fft.fft(gyro_data)[:N//2])  # Magnitude spectrum

# # Plot the frequency spectrum
# plt.figure(figsize=(8, 4))
# plt.plot(freqs, fft_magnitude)
# plt.xlabel("Frequency (Hz)")
# plt.ylabel("Magnitude")
# plt.title("Frequency Spectrum of Gyroscope Data")
# plt.grid()
# plt.show()