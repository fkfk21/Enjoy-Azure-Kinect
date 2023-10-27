import speech_recognition as sr
import numpy as np
from scipy.io import wavfile

# WAVファイルを読み込む
sample_rate, data = wavfile.read('output_1.wav')
# print(data.shape)

# Numpy配列をAudioDataに変換
audio_data = sr.AudioData(data[0:96000*4].tobytes(), sample_rate, 4)

# Recognizer クラスのインスタンスを作成
recognizer = sr.Recognizer()

# Google Web Speech APIを使用して文字起こしを試みる
with sr.AudioFile('mic_1.wav') as source:
  audio = recognizer.record(source)
  try:
      text = recognizer.recognize_google(audio, language='ja-JP')
      print('文字起こし結果:', text)
  except sr.UnknownValueError:
      print("Google Web Speech APIは音声を認識できませんでした")
  except sr.RequestError as e:
      print("Google Web Speech APIのリクエストに失敗しました; {0}".format(e))