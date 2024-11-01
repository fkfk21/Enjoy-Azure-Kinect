import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
from isort import file
from tools import get_device_number

# 録音の設定
fs = 48000  # サンプリング周波数
duration = 5  # 録音時間（秒）
channels = 7
device_num = get_device_number("Azure Kinect") # マイクロフォンアレイのデバイス番号

# マイクロフォンアレイからの録音
print("録音開始...")
audio_data = sd.rec(int(duration * fs), samplerate=fs, device=device_num,
                    channels=channels, dtype='float64')
sd.wait()  # 録音が終了するまで待つ
print("録音終了")

# 各マイクロフォンのデータを別々のファイルに保存
for i in range(audio_data.shape[1]):
    filename = f"mic_{i+1}.wav"
    write(filename, fs, audio_data[:, i])
    print(f"{filename}に保存しました")

# マイクロフォンアレイのデータをファイルに保存
filename = "mic_array.wav"
write(filename, fs, audio_data)
print(f"{filename}に保存しました")