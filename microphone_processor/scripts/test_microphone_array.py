import numpy as np
import matplotlib.pyplot as plt
import sounddevice as sd
from tools import get_device_number


# サンプリング周波数とバッファサイズの設定
fs = 48000
buffer_size = 2048
device_num = get_device_number("Azure Kinect") # マイクロフォンアレイのデバイス番号
channels = 7 # マイクロフォンアレイの数

# プロットをインタラクティブモードで表示
fig, ax = plt.subplots()
x = np.fft.rfftfreq(buffer_size, d=1/fs)
y = np.zeros_like(x)
line, = ax.plot(x, y)
ax.set_ylim(0, 20.0)
ax.set_xlim(0, fs/2)
# ax.set_xlim(0, 10000)
ax.set_xlabel('Frequency [Hz]')
ax.set_ylabel('Magnitude')

# 窓関数
window = np.hanning(buffer_size)

def callback(indata, frames, time, status):
    if status:
        print(status)
    # print(indata.shape, frames, time)
    data_windowed = indata[:, 0] * window
    spectrum = np.fft.rfft(data_windowed, n=frames)
    magnitude = np.abs(spectrum)*2 / frames
    line.set_ydata(magnitude)
    plt.draw()

stream = sd.InputStream(callback=callback, device=device_num, channels=channels,
                        samplerate=fs, blocksize=buffer_size, dtype='int16')
with stream:
    plt.show(block=True)