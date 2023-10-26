import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write

# 録音の設定
fs = 44100  # サンプリング周波数
duration = 5  # 録音時間（秒）

# マイクロフォンアレイからの録音
print("録音開始...")
audio_data = sd.rec(int(duration * fs), samplerate=fs, channels=4, dtype='float64')
sd.wait()  # 録音が終了するまで待つ
print("録音終了")

# 各マイクロフォンのデータを別々のファイルに保存
for i in range(audio_data.shape[1]):
    filename = f"mic_{i+1}.wav"
    write(filename, fs, audio_data[:, i])
    print(f"{filename}に保存しました")