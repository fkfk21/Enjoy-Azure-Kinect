from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # ros2 launch rosbridge_server rosbridge_websocket_launch.xml
  # ros2 launch azure_kinect_ros_driver driver.launch.py depth_mode:=NFOV_UNBINNED color_resolution:=720P fps:=15
  # ros2 launch enjoy_azure_kinect start.launch.py

  return LaunchDescription([
    Node(
      package='enjoy_azure_kinect',
      executable='compress_image_node',
      name='compress_image_node',
      parameters=[{'input_topic': '/rgb/image_raw'}],
      output='screen'
    ),
    Node(
      package='enjoy_azure_kinect',
      executable='compress_image_node',
      name='compress_image_node',
      parameters=[{'input_topic': '/depth/image_raw'}],
      output='screen'
    ),
    Node(
      package='enjoy_azure_kinect',
      executable='compress_image_node',
      name='compress_image_node',
      parameters=[{'input_topic': '/depth_to_rgb/image_raw'}],
      output='screen'
    ),
    Node(
      package='enjoy_azure_kinect',
      executable='compress_image_node',
      name='compress_image_node',
      parameters=[{'input_topic': '/ir/image_raw'}],
      output='screen'
    ),
    Node(
      package='enjoy_azure_kinect',
      executable='compress_image_node',
      name='compress_image_node',
      parameters=[{'input_topic': '/rgb_to_depth/image_raw'}],
      output='screen'
    ),
    Node(
      package='enjoy_azure_kinect',
      executable='hark_executor_node',
      name='hark_executor_node',
      output='screen'
    ),
  ])
