# mynteye_camera

ROS 2 Jazzy driver for the MYNT EYE D1000 stereo IR depth camera. S1030-IR support is planned.

The node uses the camera's UVC/V4L2 stereo stream by default and keeps the old MYNT-EYE-S-SDK as a build dependency/reference. This avoids the fragile SDK streaming path on Ubuntu 24.04 while preserving the official depth/disparity behavior where it matters.

## Supported Setup

- Ubuntu 24.04
- ROS 2 Jazzy
- OpenCV 4 from the system/ROS installation
- MYNT EYE D1000 exposing `1e4e:0120` (S1030-IR support planned)
- Working stereo UVC mode: `1280x480 YUYV`, split into two `640x480` mono IR images

## Build

From the repository root:

```bash
./scripts/build_legacy_sdk_ubuntu24.sh
cd mynteye_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mynteye_camera --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

The SDK is expected at `../../install` relative to this package. Override it with CMake if needed:

```bash
colcon build --symlink-install --packages-select mynteye_camera \
  --cmake-args -DMYNTEYE_SDK_ROOT=/absolute/path/to/sdk/install
```

## Launch

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mynteye_camera mynteye.launch.py device_wait_sec:=60
```

The default UVC mode requests `1280x480 YUYV`. On the tested camera this is the real stereo mode: left and right IR frames are the two `640x480` luminance halves. The `640x480` UVC mode is effectively a single image on this hardware and should not be used for depth.

## Main Topics

- `/mynteye/left/image_raw` and `/mynteye/right/image_raw`: split mono8 stereo images.
- `/mynteye/left/image_rect` and `/mynteye/right/image_rect`: rectified grayscale images used for matching.
- `/mynteye/left/camera_info` and `/mynteye/right/camera_info`: approximate pinhole camera info.
- `/mynteye/left/disparity/image_raw` and `/mynteye/right/disparity/image_raw`: `32FC1` disparity in pixels.
- `/mynteye/left/depth/image_raw` and `/mynteye/right/depth/image_raw`: default `16UC1` depth in millimeters, invalid pixels set to `0`.
- `/mynteye/points`: organized `sensor_msgs/msg/PointCloud2` with `x`, `y`, `z`, `intensity`.
- `/mynteye/debug/disparity`: `mono8` disparity preview for quick visual checks.
- `/mynteye/imu/data_raw`: IMU data when the SDK backend is used.

## TF Tree

The node publishes static transforms on `/tf_static`:

- `mynteye_link -> mynteye_left`
- `mynteye_link -> mynteye_right`
- `mynteye_left -> mynteye_points`
- `mynteye_link -> mynteye_imu`

Use `mynteye_link` as the RViz fixed frame. By default it is centered between both stereo cameras. Set `base_at_stereo_center:=false` to keep the left camera at the base frame origin.

The point cloud defaults to the same axis convention used by the official ROS 1 wrapper: OpenCV camera coordinates `(X right, Y down, Z forward)` are published as `(x=Z, y=-X, z=-Y)` in `mynteye_points`. Set `pointcloud_ros_coordinates:=false` for raw optical XYZ in the left camera frame.

## Depth Behavior

The official SDK publishes sparse/filtered depth rather than a dense map full of weak matches. For RViz and live operation, this node defaults to a less aggressive balanced preset:

- `stereo_matcher:=sgbm`: denser than the SDK BM preset while keeping the SDK SGBM parameters.
- `enable_lr_check:=false`: avoids over-pruning on this UVC stream.
- `depth_image_encoding:=16UC1`: depth in millimeters with `0` invalid.
- `depth_cleanup_kernel:=0` and `depth_min_component_area:=20`: remove tiny speckles without collapsing the map into a few patches.

For the stricter official-style BM mode, use:

```bash
ros2 launch mynteye_camera mynteye.launch.py \
  stereo_matcher:=bm \
  enable_lr_check:=true \
  depth_cleanup_kernel:=3 \
  depth_min_component_area:=120
```

See [docs/DEPTH_TUNING.md](docs/DEPTH_TUNING.md) for tuning presets and debugging steps.

## Calibration

`calibration_preset:=sdk_default` uses the legacy SDK's bundled D1000 pinhole intrinsics and extrinsics, scaled to the active UVC image size. It is good enough for live visualization, but camera-specific stereo calibration is required for measurements.

For calibrated depth, pass your own values:

```bash
ros2 launch mynteye_camera mynteye.launch.py \
  calibration_preset:=approximate \
  baseline_m:=0.12 \
  fx:=160.0 fy:=160.0 cx:=159.5 cy:=239.5
```

If `fx` and `fy` are left at `0.0`, the node estimates focal length from `horizontal_fov_deg`.

## Useful Parameters

- `backend`: `uvc` by default; `sdk` is available but less reliable on Ubuntu 24.04.
- `width`, `height`: requested UVC frame size; use `1280x480` for stereo.
- `uvc_stereo_layout`: `auto`, `side_by_side_yuyv`, or `interleaved_yuyv`.
- `swap_stereo`: swap left/right images if the device order is reversed.
- `enable_depth`: enable derived disparity, depth and point cloud products.
- `publish_pointcloud`: publish `/mynteye/points`.
- `pointcloud_stride`: point cloud pixel downsample; higher is lighter.
- `stereo_matcher`: `bm` or `sgbm`.
- `block_size`, `uniqueness_ratio`, `speckle_range`, `pre_filter_cap`: `0` means use official defaults for the selected matcher.
- `bm_pre_filter_size`, `bm_texture_threshold`: BM-specific official SDK controls.
- `enable_lr_check`, `lr_check_tolerance_px`: left-right consistency filtering.
- `disparity_median_kernel`, `depth_cleanup_kernel`, `depth_min_component_area`: post-filtering for noisy IR scenes.
- `depth_image_encoding`: `16UC1` for official-style millimeters, or `32FC1` for meters.
- `min_depth_m`, `max_depth_m`: clipping range.
- `publish_tf`, `base_frame`, `base_at_stereo_center`, `imu_x_m`, `imu_y_m`, `imu_z_m`: TF configuration.

## Troubleshooting

Check the V4L2 modes:

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
```

A correct launch log should show a `1280x480` UVC stream. If depth looks like one eye only, force the side-by-side layout:

```bash
ros2 launch mynteye_camera mynteye.launch.py uvc_stereo_layout:=side_by_side_yuyv
```

If the depth map is too noisy, start from the default BM mode and tighten filtering:

```bash
ros2 launch mynteye_camera mynteye.launch.py \
  enable_lr_check:=true \
  depth_cleanup_kernel:=3 \
  depth_min_component_area:=120 \
  lr_check_tolerance_px:=1.0 \
  max_depth_m:=5.0
```

If it is too sparse, relax filtering gradually:

```bash
ros2 launch mynteye_camera mynteye.launch.py \
  depth_cleanup_kernel:=0 \
  depth_min_component_area:=0 \
  enable_lr_check:=false \
  stereo_matcher:=sgbm
```