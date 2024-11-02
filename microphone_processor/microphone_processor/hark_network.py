import hark

import plotQuickSourceKivy
import os
from .tools import get_device_number
from ament_index_python.packages import get_package_share_directory

# 音源定位/分離パラメータ
LOWER_BOUND_FREQUENCY = 300
UPPER_BOUND_FREQUENCY = 3000
BLOCKSIZE = 512
# マイクロフォンパラメータ
DEVICE_NUM = get_device_number("Azure Kinect")
SAMPLE_RATE = 48000
CHANNELS = 7
TF_ZIP = "tf.zip"
PACKAGE_DIR = get_package_share_directory("microphone_processor")
TF_ZIP_PATH = os.path.join(PACKAGE_DIR, 'config', TF_ZIP)

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
            .add_input("A_MATRIX", TF_ZIP_PATH)
            .add_input("MUSIC_ALGORITHM", "SEVD")
            # .add_input("MUSIC_ALGORITHM", "GEVD")
            # .add_input("MUSIC_ALGORITHM", "GSVD")
            # .add_input("WINDOW_TYPE", "PAST")
            # .add_input("WINDOW_TYPE", "MIDDLE")
            .add_input("MIN_DEG", -70)
            .add_input("MAX_DEG",  70)
            .add_input("LOWER_BOUND_FREQUENCY", LOWER_BOUND_FREQUENCY)
            .add_input("UPPER_BOUND_FREQUENCY", UPPER_BOUND_FREQUENCY)
            .add_input("WINDOW", 50)
            .add_input("PERIOD", 10)
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
            .add_input("TF_CONJ_FILENAME", TF_ZIP_PATH)
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

class HARK_Recognition(hark.NetworkDef):
    '''音声認識サブネットワークに相当するクラス。
    入力として8chスペクトログラム（GHDSSの出力）を受け取り、
    音響特徴量を抽出し、Kaldiに特徴量を送信する。
    '''

    def build(self,
              network: hark.Network,
              input:   hark.DataSourceMap,
              output:  hark.DataSinkMap):

        # 必要なノードを定義する
        node_white_noise_adder = network.create(hark.node.WhiteNoiseAdder)
        node_feature_remover = network.create(hark.node.FeatureRemover)
        node_delta = network.create(hark.node.Delta)
        node_mel_filter_bank = network.create(hark.node.MelFilterBank)
        node_msls_extraction = network.create(hark.node.MSLSExtraction)
        node_pre_emphasis = network.create(hark.node.PreEmphasis)
        node_spectral_mean_normalization_incremental = network.create(
            hark.node.SpectralMeanNormalizationIncremental)
        node_speech_recognition_client = network.create(hark.node.SpeechRecognitionClient)

        # ノード間の接続（データの流れ）とパラメータを記述する
        (
            node_white_noise_adder
            .add_input("INPUT", input["INPUT"])
            .add_input("WN_LEVEL", 15)
        )
        (
            node_pre_emphasis
            .add_input("INPUT", node_white_noise_adder["OUTPUT"])
            .add_input("INPUT_TYPE", "SPECTRUM")
        )
        (
            node_mel_filter_bank
            .add_input("INPUT", node_pre_emphasis["OUTPUT"])
            .add_input("FBANK_COUNT", 40)
        )
        (
            node_msls_extraction
            .add_input("FBANK", node_mel_filter_bank["OUTPUT"])
            .add_input("SPECTRUM", node_pre_emphasis["OUTPUT"])
            .add_input("FBANK_COUNT", 40)
            .add_input("NORMALIZATION_MODE", "SPECTRAL")
            .add_input("USE_POWER", True)
        )
        (
            node_delta
            .add_input("INPUT", node_msls_extraction["OUTPUT"])
        )
        (
            node_feature_remover
            .add_input("INPUT", node_delta["OUTPUT"])
            .add_input("SELECTOR", " ".join([str(c) for c in range(40, 81 + 1)]))
        )
        (
            node_spectral_mean_normalization_incremental
            .add_input("INPUT", node_feature_remover["OUTPUT"])
            .add_input("NOT_EOF", True)  # ToDo
            .add_input("SM_HISTORY", False)  # ToDo
            .add_input("PERIOD", 1)
            .add_input("BASENAME", "")
            .add_input("OUTPUT_FNAME", "")
        )
        (
            node_speech_recognition_client
            # .add_input("FEATURES", node_feature_remover["OUTPUT"])
            # .add_input("MASKS", node_feature_remover["OUTPUT"])
            .add_input("FEATURES", node_spectral_mean_normalization_incremental["OUTPUT"])
            .add_input("MASKS", node_spectral_mean_normalization_incremental["OUTPUT"])
            .add_input("SOURCES", input["SOURCES"])
            .add_input("MFM_ENABLED", False)
            .add_input("HOST", "localhost")
            .add_input("PORT", 5530)
            .add_input("SOCKET_ENABLED", True)
        )
        (
            output
            .add_input("OUTPUT", node_feature_remover["OUTPUT"])
        )

        # ネットワークに含まれるノードの一覧をリストにする
        r = [
            node_white_noise_adder,
            node_feature_remover,
            node_delta,
            node_mel_filter_bank,
            node_msls_extraction,
            node_pre_emphasis,
            node_spectral_mean_normalization_incremental,
            node_speech_recognition_client,
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
        # node_separation = network.create(
        #     HARK_Separation,
        #     name="HARK_Separation")

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
        # (
        #     node_separation
        #     .add_input("SPEC", node_multi_fft["OUTPUT"])
        #     .add_input("SOURCES", node_localization["OUTPUT"])
        # )
        (
            node_localization_subscriber
            .add_input("INPUT", node_localization["OUTPUT"])
        )
        # (
        #     node_stream_subscriber
        #     .add_input("INPUT", node_separation["WAVEFORM"])
        # )

        # ネットワークに含まれるノードの一覧をリストにして返す
        r = [
            node_publisher,
            node_localization_subscriber,
            node_audio_stream_from_memory,
            node_multi_gain,
            node_multi_fft,
            node_localization,
            # node_stream_subscriber
        ]
        return r
