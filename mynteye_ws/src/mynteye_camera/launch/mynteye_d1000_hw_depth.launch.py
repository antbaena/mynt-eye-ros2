"""Launch file for the MYNT EYE D1000 hardware-depth node (via MYNT-EYE-D-SDK).

This uses the on-chip eSPDI depth processor instead of software SGBM.
Requires that scripts/build_d_sdk_ubuntu24.sh has been run first.

Topics published (under node namespace, default: /mynteye_d1000):
  left/image_raw        — sensor_msgs/Image  BGR8  640x480 (or 1280x720 in hd mode)
  right/image_raw       — sensor_msgs/Image  BGR8
  left/camera_info      — sensor_msgs/CameraInfo  (calibration from camera EEPROM)
  right/camera_info     — sensor_msgs/CameraInfo
  depth/image_raw       — sensor_msgs/Image  16UC1 (mm)
  depth/camera_info     — sensor_msgs/CameraInfo
  points                — sensor_msgs/PointCloud2

Usage:
  ros2 launch mynteye_camera mynteye_d1000_hw_depth.launch.py
  ros2 launch mynteye_camera mynteye_d1000_hw_depth.launch.py stream_mode:=1280x720
  ros2 launch mynteye_camera mynteye_d1000_hw_depth.launch.py ir_intensity:=6
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # -------------------------------------------------------------------
        # Camera device
        # -------------------------------------------------------------------
        DeclareLaunchArgument(
            'dev_index',
            default_value='0',
            description='D-SDK device index (0 = first detected D1000)'),

        # -------------------------------------------------------------------
        # Acquisition parameters
        # -------------------------------------------------------------------
        DeclareLaunchArgument(
            'framerate',
            default_value='30',
            description='Capture framerate (fps). Max 60 for 1280x480, max 30 for 1280x720.'),

        DeclareLaunchArgument(
            'ir_intensity',
            default_value='4',
            description='IR projector intensity [0–10]. Higher = more IR light, better depth indoors.'),

        DeclareLaunchArgument(
            'ir_depth_only',
            default_value='false',
            description='When true, only the depth stream is active (no color). Saves bandwidth.'),

        DeclareLaunchArgument(
            'stream_mode',
            default_value='1280x480',
            description='Stream resolution. Options: 1280x480 (640x480/eye), 1280x720 (HD), 2560x720.'),

        # -------------------------------------------------------------------
        # Frame IDs
        # -------------------------------------------------------------------
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='mynteye_link',
            description='Root TF frame for the camera body.'),

        DeclareLaunchArgument(
            'left_frame_id',
            default_value='mynteye_left',
            description='TF frame for the left camera / depth reference.'),

        DeclareLaunchArgument(
            'right_frame_id',
            default_value='mynteye_right',
            description='TF frame for the right camera.'),

        DeclareLaunchArgument(
            'depth_frame_id',
            default_value='mynteye_depth',
            description='TF frame published with the depth image (co-located with left camera).'),

        # -------------------------------------------------------------------
        # Node
        # -------------------------------------------------------------------
        Node(
            package='mynteye_camera',
            executable='mynteye_d1000_node',
            name='mynteye_d1000',
            namespace='mynteye_d1000',
            output='screen',
            parameters=[{
                'dev_index':     LaunchConfiguration('dev_index'),
                'framerate':     LaunchConfiguration('framerate'),
                'ir_intensity':  LaunchConfiguration('ir_intensity'),
                'ir_depth_only': LaunchConfiguration('ir_depth_only'),
                'stream_mode':   LaunchConfiguration('stream_mode'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'left_frame_id': LaunchConfiguration('left_frame_id'),
                'right_frame_id': LaunchConfiguration('right_frame_id'),
                'depth_frame_id': LaunchConfiguration('depth_frame_id'),
            }],
        ),
    ])
