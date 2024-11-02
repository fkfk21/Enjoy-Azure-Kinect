#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import threading
# import tempfile
from std_msgs.msg import Float64MultiArray

import numpy as np
import sounddevice as sd

import hark

from ..microphone_processor.hark_network import (
  HARK_Main, SAMPLE_RATE, BLOCKSIZE, CHANNELS, DEVICE_NUM
)


class HARKExecutor(Node):

    def __init__(self):
        super().__init__('hark_executor_node')
        self.publisher = self.create_publisher(Float64MultiArray, 'angle', 10)

        # empty timer callback
        self.timer = self.create_timer(0.1, self.timer_callback)

        """
            hark settings
        """
        # filename = tempfile.mktemp(prefix='practice3-1a_',
        #                            suffix='.wav',
        #                            dir='')

        # メインネットワークを構築
        self.hark_network = hark.Network.from_networkdef(HARK_Main,
                                                         name="HARK_Main")

        # メインネットワークへの入出力を構築
        self.audio_publisher = self.hark_network.query_nodedef("Publisher")
        localization_subscriber = self.hark_network.query_nodedef(
            "LocalizationSubscriber")
        stream_subscriber = self.hark_network.query_nodedef("StreamSubscriber")

        localization_subscriber.receive = self.hark_localization_received_callback
        stream_subscriber.receive = self.hark_stream_received_callback

        def callback(indata, frames, time, status):
            # print(indata.shape, time.currentTime)
            self.audio_publisher.push(indata.T)

        # ネットワーク実行用スレッドを立ち上げ
        self.network_thread = threading.Thread(
            target=self.hark_network.execute)
        self.network_thread.start()

        # ネットワーク実行
        self.stream = sd.InputStream(samplerate=SAMPLE_RATE,
                                     blocksize=BLOCKSIZE,
                                     device=DEVICE_NUM,
                                     dtype=np.int16,
                                     channels=CHANNELS,
                                     callback=callback)
        self.stream.start()

    def hark_localization_received_callback(self, data):
        """
        data: List[harklib.Source]
        harklib.Source: id, power, x = [x, y, z]
                        'compare_mode', 'count', 'first_found_count', 'id',
                        'last_found_count', 'mic_id', 'power', 'remaining',
                        'sub_id', 'tfindex', 'tfindex_max', 'tfindex_min', 'x'
        """
        # for d in data:
        #     print(d.id, d.count, d.power)
        # print(dir(d))
        # last[0] = t
        msg = Float64MultiArray()
        arr = []
        for d in data:
            x, y = d.x[0], d.x[1]
            arr.append(np.arctan2(y, x))
            print(np.rad2deg(np.arctan2(y, x)))
        msg.data = arr
        self.publisher.publish(msg)
        pass

    def hark_stream_received_callback(self, data):
        """
        data: List[np.ndarray]
        """
        # for d in data:
        #     # print(d.id, d.x, d.power)
        #     print(dir(d))
        # for id, arr in data.items():
        #     print(id, arr.shape)
        pass

    def timer_callback(self):
        pass

    def __del__(self):
        if hasattr(self, 'stream'):
            self.stream.stop()
            self.stream.close()
        if hasattr(self, 'hark_network'):
            self.hark_network.stop()
        if hasattr(self, 'audio_publisher'):
            self.audio_publisher.close()
        if hasattr(self, 'network_thread'):
            self.network_thread.join()


def main(args=None):
    # ros node init
    rclpy.init(args=args)
    hark_executor_node = HARKExecutor()
    rclpy.spin(hark_executor_node)
    hark_executor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # parser = argparse.ArgumentParser(add_help=False)
    # parser.add_argument(
    #     '-l', '--list-devices', action='store_true',
    #     help='show list of audio devices and exit')
    # args, remaining = parser.parse_known_args()
    # if args.list_devices:
    #     print(sd.query_devices())
    #     parser.exit(0)
    # parser = argparse.ArgumentParser(
    #     description=__doc__,
    #     formatter_class=argparse.RawDescriptionHelpFormatter,
    #     parents=[parser])
    # parser.add_argument(
    #     'filename', nargs='?', metavar='FILENAME',
    #     help='audio file to store recording to')
    # parser.add_argument(
    #     '-t', '--subtype', type=str, help='sound file subtype (e.g. "PCM_24")')
    # args = parser.parse_args(remaining)

    main()

# end of file
