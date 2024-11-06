from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # ros2 launch rosbridge_server rosbridge_websocket_launch.xml
  # ros2 launch azure_kinect_ros_driver driver.launch.py
  #                             depth_mode:=NFOV_UNBINNED color_resolution:=720P fps:=15
  # ros2 launch enjoy_azure_kinect start.launch.py

  return LaunchDescription([
    Node(
      package='image_processor',
      executable='compress_image_node',
      name='compress_image_node',
      output='screen'
    ),
    Node(
      package='image_processor',
      executable='extract_target_node',
      name='extract_target_node',
      output='screen'
    ),
  ])
