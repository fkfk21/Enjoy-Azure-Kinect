#!/usr/bin/env python

import threading
import time
import argparse
import tempfile

import numpy as np
import sounddevice as sd
import soundfile as sf
from numpy.lib.stride_tricks import sliding_window_view

import hark

import plotQuickWaveformKivy
import plotQuickSpecKivy
import plotQuickMusicSpecKivy
import plotQuickSourceKivy

from tools import get_device_number

# 音源定位/分離パラメータ
LOWER_BOUND_FREQUENCY = 300
UPPER_BOUND_FREQUENCY = 3000
BLOCKSIZE = 2048
# マイクロフォンパラメータ
DEVICE_NUM = get_device_number("Azure Kinect")
SAMPLE_RATE = 48000
CHANNELS = 7

class HARK_Localization(hark.NetworkDef):
    '''音源定位サブネットワークに相当するクラス。
    入力として7ch音響信号を受け取り、
    フーリエ変換、MUSIC法による音源定位、音源追跡を行い、
    その結果を図示する。
    '''

    def build(self,
              network: hark.Network,
              input:   hark.DataSourceMap,
              output:  hark.DataSinkMap):
        
        # 必要なノードを定義する
        node_localize_music = network.create(hark.node.LocalizeMUSIC)
        node_cm_identity_matrix = network.create(
            hark.node.CMIdentityMatrix,
            dispatch=hark.RepeatDispatcher)
        node_source_tracker = network.create(hark.node.SourceTracker)
        node_source_interval_extender = network.create(
            hark.node.SourceIntervalExtender)
        node_plotsource_kivy = network.create(
            plotQuickSourceKivy.plotQuickSourceKivy)

        # ノード間の接続（データの流れ）とパラメータを記述する
        (
            node_localize_music
            .add_input("INPUT", input["INPUT"])
            .add_input("NOISECM", node_cm_identity_matrix["OUTPUT"])
            .add_input("A_MATRIX", "tf.zip")
            .add_input("MUSIC_ALGORITHM", "SEVD")
            # .add_input("MUSIC_ALGORITHM", "GEVD")
            # .add_input("MUSIC_ALGORITHM", "GSVD")
            # .add_input("WINDOW_TYPE", "PAST")
            # .add_input("WINDOW_TYPE", "MIDDLE")
            .add_input("MIN_DEG", -180)
            .add_input("MAX_DEG",  180)
            .add_input("LOWER_BOUND_FREQUENCY", LOWER_BOUND_FREQUENCY)
            .add_input("UPPER_BOUND_FREQUENCY", UPPER_BOUND_FREQUENCY)
            .add_input("WINDOW", 50)
            .add_input("PERIOD", 1)
            .add_input("NUM_SOURCE", 2)
            .add_input("DEBUG", False)
        )
        (
            node_cm_identity_matrix
            .add_input("NB_CHANNELS", CHANNELS)
            .add_input("LENGTH", BLOCKSIZE)
        )
        (
            node_source_tracker
            .add_input("INPUT", node_localize_music["OUTPUT"])
            .add_input("THRESH", 32)
            .add_input("PAUSE_LENGTH", 1200.0)
            .add_input("MIN_SRC_INTERVAL", 20.0)
            .add_input("MIN_ID", 0)
            .add_input("DEBUG", False)
        )
        (
            node_source_interval_extender
            .add_input("SOURCES", node_source_tracker["OUTPUT"])
        )
        (
            node_plotsource_kivy
            .add_input("SOURCES", node_source_tracker["OUTPUT"])
        )
        (
            output
            .add_input("OUTPUT", node_source_interval_extender["OUTPUT"])
        )

        # ネットワークに含まれるノードの一覧をリストにする
        r = [
            node_localize_music,
            node_cm_identity_matrix,
            node_source_tracker,
            node_plotsource_kivy,
        ]

        # ノード一覧のリストを返す
        return r
    
class HARK_Separation(hark.NetworkDef):
    '''音源分離サブネットワークに相当するクラス。
    入力として7chスペクトログラムと音源定位結果を受け取り、
    GHDSSによる音源分離を行い、その結果を出力する。
    '''

    def build(self,
              network: hark.Network,
              input:   hark.DataSourceMap,
              output:  hark.DataSinkMap):

        # 必要なノードを定義する
        node_ghdss = network.create(hark.node.GHDSS)
        node_synthesize = network.create(hark.node.Synthesize)
        node_save_wave_pcm = network.create(hark.node.SaveWavePCM)

        # ノード間の接続（データの流れ）とパラメータを記述する
        (
            node_ghdss
            .add_input("INPUT_FRAMES", input["SPEC"])
            .add_input("INPUT_SOURCES", input["SOURCES"])
            .add_input("TF_CONJ_FILENAME", "tf.zip")
            .add_input("SAMPLING_RATE", SAMPLE_RATE)
            .add_input("LENGTH", BLOCKSIZE)
            .add_input("LOWER_BOUND_FREQUENCY", 0)
            .add_input("UPPER_BOUND_FREQUENCY", 8000)
        )
        (
            node_synthesize
            .add_input("INPUT", node_ghdss["OUTPUT"])
            .add_input("LENGTH", BLOCKSIZE)
            .add_input("ADVANCE", 160)
            .add_input("SAMPLING_RATE", SAMPLE_RATE)
            .add_input("WINDOW", "HAMMING")
        )
        (
            node_save_wave_pcm
            .add_input("INPUT", node_synthesize["OUTPUT"])
            .add_input("SAMPLING_RATE", SAMPLE_RATE)
        )
        (
            output
            .add_input("SPECTRUM", node_ghdss["OUTPUT"])
            .add_input("WAVEFORM", node_save_wave_pcm["OUTPUT"])
        )

        # ネットワークに含まれるノードの一覧をリストにする
        r = [
            node_ghdss,
            node_synthesize,
            node_save_wave_pcm,
        ]

        # ノード一覧のリストを返す
        return r

class HARK_Main(hark.NetworkDef):
    '''メインネットワークに相当するクラス。
    入力として7ch音響信号を受け取り、
    フーリエ変換、MUSIC法による音源定位、音源追跡を行い、
    その結果を図示する。
    '''

    def build(self,
              network: hark.Network,
              input:   hark.DataSourceMap,
              output:  hark.DataSinkMap):

        # 必要なノードを定義する。
        # メインネットワークには全体の入出力を扱う
        # Publisher と Subscriber が必要。
        # さらに、AudioStreamFromMemoryノード、
        # MultiFFTノード、音源定位サブネットワークを定義する。
        node_publisher = network.create(
            hark.node.PublishData,
            dispatch=hark.RepeatDispatcher,
            name="Publisher")
        node_localization_subscriber = network.create(
            hark.node.SubscribeData,
            name="LocalizationSubscriber")
        # node_stream_subscriber = network.create(
        #     hark.node.SubscribeData,
        #     name="StreamSubscriber")

        node_audio_stream_from_memory = network.create(
            hark.node.AudioStreamFromMemory,
            dispatch=hark.TriggeredMultiShotDispatcher)
        node_multi_gain = network.create(hark.node.MultiGain)
        node_multi_fft = network.create(hark.node.MultiFFT)
        node_localization = network.create(
            HARK_Localization,
            name="HARK_Localization")
        node_separation = network.create(
            HARK_Separation,
            name="HARK_Separation")

        # ノード間の接続（データの流れ）を記述する
        (
            node_audio_stream_from_memory
            .add_input("INPUT", node_publisher["OUTPUT"])
            .add_input("CHANNEL_COUNT", CHANNELS)
            .add_input("LENGTH", BLOCKSIZE)
        )
        (
            node_multi_gain
            .add_input("INPUT", node_audio_stream_from_memory["AUDIO"])
            .add_input("GAIN", 2**3)
        )
        (
            node_multi_fft
            .add_input("INPUT", node_audio_stream_from_memory["AUDIO"])
            # .add_input("INPUT", node_multi_gain[""])
            .add_input("LENGTH", BLOCKSIZE)
            .add_input("WINDOW_LENGTH", BLOCKSIZE)
            .add_input("WINDOW", "HANNING")
        )
        (
            node_localization
            .add_input("INPUT", node_multi_fft["OUTPUT"])
        )
        (
            node_separation
            .add_input("SPEC", node_multi_fft["OUTPUT"])
            .add_input("SOURCES", node_localization["OUTPUT"])
        )
        (
            node_localization_subscriber
            .add_input("INPUT", node_localization["OUTPUT"])
        )

        # ネットワークに含まれるノードの一覧をリストにして返す
        r = [
            node_publisher,
            node_localization_subscriber,
            node_audio_stream_from_memory,
            node_multi_gain,
            node_multi_fft,
            node_localization,
        ]
        return r


def main():

    def int_or_str(text):
        """Helper function for argument parsing."""
        try:
            return int(text)
        except ValueError:
            return text

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        '-l', '--list-devices', action='store_true',
        help='show list of audio devices and exit')
    args, remaining = parser.parse_known_args()
    if args.list_devices:
        print(sd.query_devices())
        parser.exit(0)
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[parser])
    parser.add_argument(
        'filename', nargs='?', metavar='FILENAME',
        help='audio file to store recording to')
    parser.add_argument(
        '-t', '--subtype', type=str, help='sound file subtype (e.g. "PCM_24")')
    args = parser.parse_args(remaining)

    if args.filename is None:
        args.filename = tempfile.mktemp(prefix='practice3-1a_',
                                        suffix='.wav', dir='')
    


    # メインネットワークを構築
    network = hark.Network.from_networkdef(HARK_Main, name="HARK_Main")

    # メインネットワークへの入出力を構築
    publisher = network.query_nodedef("Publisher")
    localization_subscriber = network.query_nodedef("LocalizationSubscriber")

    last = [time.time()]
    def received(data):
        """
        data: List[harklib.Source]
        harklib.Source: id, power, x = [x, y, z]
        """
        for d in data:
            print(d.id, d.x, d.power)
        # last[0] = t
        pass

    localization_subscriber.receive = received

    def callback(indata, frames, time, status):
        # print(indata.shape, time.currentTime)
        publisher.push(indata.T)

    # ネットワーク実行用スレッドを立ち上げ
    th = threading.Thread(target=network.execute)
    th.start()

    # ネットワーク実行
    try:
        with sd.InputStream(samplerate=SAMPLE_RATE, blocksize=BLOCKSIZE,
                            device=DEVICE_NUM, dtype=np.int16,
                            channels=CHANNELS, callback=callback) as stream:
            print('#' * 75)
            print('press Ctrl+C to stop the recording')
            print('#' * 75)
            th.join()
            # while th.is_alive():
            #     time.sleep(0.1)
            
    except KeyboardInterrupt:
        print('\nRecording finished: ' + repr(args.filename))
        parser.exit(0)
    except Exception as e:
        parser.exit(type(e).__name__ + ': ' + str(e))

    # 終了処理
    finally:
        publisher.close()
        network.stop()
        th.join()


if __name__ == '__main__':
    main()

# end of file
