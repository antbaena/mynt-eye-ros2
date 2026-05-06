"""Launch file for the MYNT EYE S1030-IR stereo camera (UVC backend + EEPROM calibration).

The S1030-IR outputs a 752x480 YUYV stream where each row interleaves left and right
camera pixels. This launch file configures the driver accordingly:
  - uvc_stereo_layout:=interleaved_yuyv  (explicit layout, avoids auto-detect ambiguity)
  - calibration_preset:=sdk_eeprom       (reads real intrinsics from camera EEPROM via S-SDK)
  - width:=752 height:=480               (native S1030-IR resolution)
  - num_disparities:=128                 (covers 0.3m–15m with fx~355, baseline~120mm)
  - block_size:=9                        (good for 752x480 IR imagery)
  - min_depth_m:=0.3 max_depth_m:=15.0  (S1030-IR rated range)

If S-SDK is unavailable at runtime, the node falls back to sdk_s1030_default which
uses hardcoded EEPROM values measured from a reference unit.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # --- Camera / transport ---
        DeclareLaunchArgument('backend',            default_value='uvc'),
        DeclareLaunchArgument('video_device',       default_value=''),
        DeclareLaunchArgument('width',              default_value='752'),
        DeclareLaunchArgument('height',             default_value='480'),
        DeclareLaunchArgument('fps',                default_value='60'),
        DeclareLaunchArgument('device_wait_sec',    default_value='15'),
        DeclareLaunchArgument('uvc_stereo_layout',  default_value='interleaved_yuyv'),
        DeclareLaunchArgument('swap_stereo',        default_value='false'),

        # --- Calibration ---
        # sdk_eeprom: reads per-unit intrinsics from camera EEPROM at startup.
        # Falls back to sdk_s1030_default (hardcoded EEPROM values from a reference unit)
        # if the S-SDK cannot be opened (e.g., S-SDK not installed).
        DeclareLaunchArgument('calibration_preset', default_value='sdk_eeprom'),
        DeclareLaunchArgument('baseline_m',         default_value='0.12088'),

        # --- Stereo / depth ---
        DeclareLaunchArgument('enable_depth',               default_value='true'),
        DeclareLaunchArgument('publish_pointcloud',         default_value='true'),
        DeclareLaunchArgument('publish_debug_images',       default_value='true'),
        DeclareLaunchArgument('publish_rectified_images',   default_value='true'),
        DeclareLaunchArgument('rectify_stereo',             default_value='true'),
        DeclareLaunchArgument('stereo_matcher',             default_value='sgbm'),
        DeclareLaunchArgument('min_disparity',              default_value='0'),
        DeclareLaunchArgument('num_disparities',            default_value='128'),
        # block_size=9 works well for 752x480 IR images; increase for noisier scenes
        DeclareLaunchArgument('block_size',                 default_value='9'),
        DeclareLaunchArgument('uniqueness_ratio',           default_value='10'),
        DeclareLaunchArgument('speckle_window_size',        default_value='100'),
        DeclareLaunchArgument('speckle_range',              default_value='32'),
        DeclareLaunchArgument('disp12_max_diff',            default_value='1'),
        DeclareLaunchArgument('pre_filter_cap',             default_value='63'),
        DeclareLaunchArgument('disparity_median_kernel',    default_value='3'),
        DeclareLaunchArgument('depth_cleanup_kernel',       default_value='5'),
        DeclareLaunchArgument('depth_min_component_area',   default_value='20'),
        DeclareLaunchArgument('min_depth_m',                default_value='0.3'),
        DeclareLaunchArgument('max_depth_m',                default_value='15.0'),
        DeclareLaunchArgument('depth_image_encoding',       default_value='16UC1'),

        # --- CLAHE preprocessing (helps with IR imagery) ---
        DeclareLaunchArgument('stereo_preprocess',      default_value='true'),
        DeclareLaunchArgument('clahe_clip_limit',       default_value='2.0'),
        DeclareLaunchArgument('clahe_tile_grid_size',   default_value='8'),

        # --- LR consistency check ---
        DeclareLaunchArgument('enable_lr_check',         default_value='false'),
        DeclareLaunchArgument('lr_check_tolerance_px',   default_value='2.0'),

        # --- Point cloud ---
        DeclareLaunchArgument('pointcloud_stride',          default_value='2'),
        DeclareLaunchArgument('pointcloud_ros_coordinates', default_value='true'),

        # --- TF / frame IDs ---
        DeclareLaunchArgument('publish_tf',         default_value='true'),
        DeclareLaunchArgument('base_frame',         default_value='mynteye_link'),
        DeclareLaunchArgument('base_at_stereo_center', default_value='true'),
        DeclareLaunchArgument('left_frame',         default_value='mynteye_left'),
        DeclareLaunchArgument('right_frame',        default_value='mynteye_right'),
        DeclareLaunchArgument('points_frame',       default_value='mynteye_points'),
        DeclareLaunchArgument('imu_frame',          default_value='mynteye_imu'),
        DeclareLaunchArgument('imu_x_m',            default_value='0.0'),
        DeclareLaunchArgument('imu_y_m',            default_value='0.0'),
        DeclareLaunchArgument('imu_z_m',            default_value='0.0'),

        Node(
            package='mynteye_camera',
            executable='mynteye_camera_node',
            name='mynteye_camera_node',
            output='screen',
            parameters=[{
                'backend':                  LaunchConfiguration('backend'),
                'video_device':             LaunchConfiguration('video_device'),
                'width':                    LaunchConfiguration('width'),
                'height':                   LaunchConfiguration('height'),
                'fps':                      LaunchConfiguration('fps'),
                'device_wait_sec':          LaunchConfiguration('device_wait_sec'),
                'split_stereo':             True,
                'uvc_stereo_layout':        LaunchConfiguration('uvc_stereo_layout'),
                'swap_stereo':              LaunchConfiguration('swap_stereo'),
                'enable_depth':             LaunchConfiguration('enable_depth'),
                'publish_pointcloud':       LaunchConfiguration('publish_pointcloud'),
                'publish_debug_images':     LaunchConfiguration('publish_debug_images'),
                'publish_rectified_images': LaunchConfiguration('publish_rectified_images'),
                'rectify_stereo':           LaunchConfiguration('rectify_stereo'),
                'calibration_preset':       LaunchConfiguration('calibration_preset'),
                'baseline_m':              LaunchConfiguration('baseline_m'),
                'stereo_preprocess':        LaunchConfiguration('stereo_preprocess'),
                'clahe_clip_limit':         LaunchConfiguration('clahe_clip_limit'),
                'clahe_tile_grid_size':     LaunchConfiguration('clahe_tile_grid_size'),
                'min_depth_m':              LaunchConfiguration('min_depth_m'),
                'max_depth_m':              LaunchConfiguration('max_depth_m'),
                'stereo_matcher':           LaunchConfiguration('stereo_matcher'),
                'min_disparity':            LaunchConfiguration('min_disparity'),
                'num_disparities':          LaunchConfiguration('num_disparities'),
                'block_size':               LaunchConfiguration('block_size'),
                'uniqueness_ratio':         LaunchConfiguration('uniqueness_ratio'),
                'speckle_window_size':      LaunchConfiguration('speckle_window_size'),
                'speckle_range':            LaunchConfiguration('speckle_range'),
                'disp12_max_diff':          LaunchConfiguration('disp12_max_diff'),
                'pre_filter_cap':           LaunchConfiguration('pre_filter_cap'),
                'disparity_median_kernel':  LaunchConfiguration('disparity_median_kernel'),
                'depth_cleanup_kernel':     LaunchConfiguration('depth_cleanup_kernel'),
                'depth_min_component_area': LaunchConfiguration('depth_min_component_area'),
                'depth_image_encoding':     LaunchConfiguration('depth_image_encoding'),
                'enable_lr_check':          LaunchConfiguration('enable_lr_check'),
                'lr_check_tolerance_px':    LaunchConfiguration('lr_check_tolerance_px'),
                'pointcloud_stride':        LaunchConfiguration('pointcloud_stride'),
                'pointcloud_ros_coordinates': LaunchConfiguration('pointcloud_ros_coordinates'),
                'publish_tf':               LaunchConfiguration('publish_tf'),
                'base_frame':               LaunchConfiguration('base_frame'),
                'base_at_stereo_center':    LaunchConfiguration('base_at_stereo_center'),
                'left_frame':               LaunchConfiguration('left_frame'),
                'right_frame':              LaunchConfiguration('right_frame'),
                'points_frame':             LaunchConfiguration('points_frame'),
                'imu_frame':                LaunchConfiguration('imu_frame'),
                'imu_x_m':                  LaunchConfiguration('imu_x_m'),
                'imu_y_m':                  LaunchConfiguration('imu_y_m'),
                'imu_z_m':                  LaunchConfiguration('imu_z_m'),
            }],
        ),
    ])
