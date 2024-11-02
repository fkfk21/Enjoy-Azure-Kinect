#! /usr/bin/env python

import sounddevice as sd
from microphone_processor.tools import get_device_number

def main(args=None):
  # rclpy.init(args=args)

  print(sd.query_devices())
  print("Device number of Azure Kinect: ", get_device_number("Azure Kinect"))

  # rclpy.shutdown()


if __name__ == '__main__':
    main()


