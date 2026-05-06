# mynteye_camera

ROS 2 Jazzy driver for the **MYNT EYE D1000** and **MYNT EYE S1030-IR** stereo cameras on Ubuntu 24.04.

Two nodes are provided:

| Node | Camera | Depth method | Launch file |
|------|--------|-------------|-------------|
| `mynteye_d1000_node` | D1000 | **Hardware eSPDI** (on-chip, 0.32–7 m) | `mynteye_d1000_hw_depth.launch.py` |
| `mynteye_camera_node` | D1000 / S1030-IR | Software SGBM (CPU, configurable range) | `mynteye_s1030.launch.py` / `mynteye.launch.py` |

---

## Camera hardware

### MYNT EYE D1000

| Property | Value |
|----------|-------|
| USB ID | `1e4e:0120` |
| V4L2 name | `MYNT EYE: MYNT-EYE-D1000` |
| Sensor | Dual IR (grayscale), stereo with eSPDI depth processor |
| Stream modes (D-SDK) | `640×480`, `1280×480`★, `1280×720`, `2560×720` |
| Color formats | YUYV, MJPG |
| Max framerate | 60 fps (30 fps for 2560×720) |
| Hardware depth range | 0.32 – 7 m (<2 % error) |
| Baseline | ~120 mm (read from EEPROM) |

★ Default for `mynteye_d1000_node` — side-by-side stereo, 640×480 per eye.

### MYNT EYE S1030-IR

| Property | Value |
|----------|-------|
| USB ID | `04b4:4722` |
| V4L2 name | `MYNT-EYE-S1030` |
| Sensor | Dual IR (grayscale), wide-angle stereo |
| V4L2 mode | `752×480 YUYV @ 60 fps` (only one mode) |
| Stereo layout | **Interleaved YUYV** → 752×480 per eye |
| Software depth range | 0.3 – 15 m (SGBM) |
| Baseline | ~120.9 mm (EEPROM: −120.884 mm) |
| Calibration | Read from EEPROM via S-SDK (`sdk_eeprom` preset) |

---

## Build

### Dependencies

```bash
sudo apt update && sudo apt install -y \
  git build-essential cmake pkg-config \
  ros-jazzy-desktop ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-tf2 ros-jazzy-tf2-ros \
  libopencv-dev libusb-dev libusb-1.0-0-dev
```

### udev rules

```bash
# D1000
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1e4e", ATTR{idProduct}=="0120", MODE="0666", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-mynteye-d1000.rules
# S1030-IR
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04b4", ATTR{idProduct}=="4722", MODE="0666", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-mynteye-s1030.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG plugdev $USER   # re-login required
```

### Build SDKs

From the repository root (`mynt-eye-ros2/`):

```bash
# MYNT-EYE-S-SDK (required for both nodes)
./scripts/build_legacy_sdk_ubuntu24.sh    # → ./install/

# MYNT-EYE-D-SDK (required only for mynteye_d1000_node)
./scripts/build_d_sdk_ubuntu24.sh         # → ./d_install/
```

### Build the ROS 2 package

```bash
cd mynteye_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mynteye_camera \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

---

## D1000 — Hardware depth node

### Launch

```bash
ros2 launch mynteye_camera mynteye_d1000_hw_depth.launch.py
```

### Topics

| Topic | Message type | Description |
|-------|-------------|-------------|
| `left/image_raw` | `Image` (BGR8) | Left camera |
| `right/image_raw` | `Image` (BGR8) | Right camera |
| `left/camera_info` | `CameraInfo` | EEPROM intrinsics |
| `right/camera_info` | `CameraInfo` | EEPROM intrinsics |
| `depth/image_raw` | `Image` (16UC1, mm) | Hardware depth (eSPDI) |
| `depth/camera_info` | `CameraInfo` | Depth camera info |
| `points` | `PointCloud2` | XYZ point cloud (m) |

### Launch parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dev_index` | `0` | Device index |
| `framerate` | `30` | FPS (0–60; max 30 for 2560×720) |
| `ir_intensity` | `4` | IR emitter power 0–10 |
| `ir_depth_only` | `false` | Disable color; only depth (needs fps ≥ 30) |
| `stream_mode` | `1280x480` | `640x480`, `1280x480`, `1280x720`, `2560x720` |
| `base_frame_id` | `mynteye_link` | TF base frame |
| `left_frame_id` | `mynteye_left` | Left camera frame |
| `right_frame_id` | `mynteye_right` | Right camera frame |
| `depth_frame_id` | `mynteye_depth` | Depth frame (= left) |

### Stream mode guide

| `stream_mode` | Per-eye resolution | Right color | Max fps |
|---------------|--------------------|-------------|---------|
| `640x480` | 640×480 | no | 60 |
| `1280x480` | 640×480 | **yes** | 60 |
| `1280x720` | 1280×720 | no | 60 |
| `2560x720` | 1280×720 | **yes** | **30** |

> The eSPDI depth processor always outputs at the left camera resolution.

### Depth encoding

`depth/image_raw` is 16UC1 with values in **mm**. Invalid pixels are `0`.
eSPDI 14-bit range: 0–16383 mm. Typical valid range: 320–7000 mm.

---

## S1030-IR — Software SGBM depth node

### Launch

```bash
ros2 launch mynteye_camera mynteye_s1030.launch.py
```

At startup the node calls the S-SDK to read real intrinsics from the camera EEPROM and configures `stereoRectify` accordingly. No manual calibration needed.

### Topics

| Topic | Message type | Description |
|-------|-------------|-------------|
| `mynteye/left/image_raw` | `Image` (mono8) | Left IR image |
| `mynteye/right/image_raw` | `Image` (mono8) | Right IR image |
| `mynteye/left/image_rect` | `Image` (mono8) | Rectified left |
| `mynteye/right/image_rect` | `Image` (mono8) | Rectified right |
| `mynteye/left/camera_info` | `CameraInfo` | Left intrinsics |
| `mynteye/right/camera_info` | `CameraInfo` | Right intrinsics |
| `mynteye/left/disparity/image_raw` | `Image` (32FC1, px) | Left disparity |
| `mynteye/right/disparity/image_raw` | `Image` (32FC1, px) | Right disparity |
| `mynteye/left/depth/image_raw` | `Image` (16UC1, mm) | SGBM depth |
| `mynteye/right/depth/image_raw` | `Image` (16UC1, mm) | Right SGBM depth |
| `mynteye/points` | `PointCloud2` | XYZ + intensity |
| `mynteye/debug/disparity` | `Image` (mono8) | Disparity preview |
| `mynteye/imu/data_raw` | `Imu` | IMU (SDK backend only) |

### Key launch parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `calibration_preset` | `sdk_eeprom` | `sdk_eeprom` (EEPROM read at startup), `sdk_s1030_default` (hardcoded reference values) |
| `backend` | `uvc` | `uvc` (V4L2) or `sdk` (S-SDK streaming) |
| `uvc_stereo_layout` | `interleaved_yuyv` | Always use `interleaved_yuyv` for S1030-IR |
| `width` / `height` | `752` / `480` | Native S1030-IR resolution |
| `fps` | `60` | Frame rate |
| `stereo_matcher` | `sgbm` | `sgbm` or `bm` |
| `num_disparities` | `128` | Must be multiple of 16; covers full range at fx≈355 |
| `block_size` | `9` | SAD window size (odd, ≥3) |
| `min_depth_m` | `0.3` | Near clip (m) |
| `max_depth_m` | `15.0` | Far clip (m) |
| `min_disparity` | `0` | Min disparity (keep at 0) |
| `enable_lr_check` | `false` | Left-right consistency filtering |
| `depth_cleanup_kernel` | `5` | Morphological cleanup kernel (0 = off) |
| `stereo_preprocess` | `true` | CLAHE contrast enhancement |
| `clahe_clip_limit` | `2.0` | CLAHE clip |
| `swap_stereo` | `false` | Swap left/right if inverted |
| `pointcloud_stride` | `2` | Point cloud decimation |

### Calibration details

`sdk_eeprom` reads these values from your specific camera at startup:

```
LEFT  fx=355.41  fy=356.80  cx=379.13  cy=245.84  (752×480)
      k1=-0.2470  k2=0.0428  p1=-0.0008  p2=0.0006
RIGHT fx=358.86  fy=358.84  cx=371.82  cy=233.52
      k1=-0.2632  k2=0.0494  p1=-0.0006  p2=-0.0007
baseline = 120.88 mm
```

If the S-SDK cannot open the camera (e.g., device busy), it falls back to `sdk_s1030_default` which uses the reference values above hardcoded.

### Read EEPROM calibration standalone

```bash
cd /path/to/mynt-eye-ros2
g++ -std=c++17 scripts/read_s_sdk_calib.cpp \
    -I install/include $(pkg-config --cflags --libs opencv4) \
    -L install/lib -lmynteye -Wl,-rpath,install/lib \
    -o /tmp/read_s_sdk_calib
LD_LIBRARY_PATH=install/lib /tmp/read_s_sdk_calib
```

---

## TF tree

Both nodes publish on `/tf_static`:

```
mynteye_link  (stereo midpoint, or left camera if base_at_stereo_center:=false)
  ├── mynteye_left
  ├── mynteye_right
  ├── mynteye_points   (= mynteye_left origin)
  └── mynteye_imu      (mynteye_camera_node only)
```

Use `mynteye_link` as the **Fixed Frame** in RViz.

Point cloud axis convention (default `pointcloud_ros_coordinates:=true`):
- ROS: `x=forward, y=left, z=up`  → `x_ros = Z_cam`, `y_ros = -X_cam`, `z_ros = -Y_cam`

Set `pointcloud_ros_coordinates:=false` for raw optical coordinates (X right, Y down, Z forward).

---

## Depth tuning (S1030-IR SGBM)

Disparity at distance `d` (meters): `disp = fx × baseline / d`

| Distance | Expected disparity (fx=355, b=120.9 mm) |
|----------|-----------------------------------------|
| 0.5 m | 85.9 px |
| 1 m | 43.0 px |
| 5 m | 8.6 px |
| 15 m | 2.9 px |

`num_disparities=128` covers 0.33 m (∞ disparity) to infinity.

For stricter, sparser depth (less noise, fewer valid pixels):

```bash
ros2 launch mynteye_camera mynteye_s1030.launch.py \
  stereo_matcher:=bm \
  enable_lr_check:=true \
  lr_check_tolerance_px:=1.0 \
  depth_cleanup_kernel:=7 \
  max_depth_m:=8.0
```

For denser depth (more valid pixels, more noise at range):

```bash
ros2 launch mynteye_camera mynteye_s1030.launch.py \
  stereo_matcher:=sgbm \
  enable_lr_check:=false \
  depth_cleanup_kernel:=0 \
  depth_min_component_area:=0
```

See [docs/DEPTH_TUNING.md](docs/DEPTH_TUNING.md) for detailed presets.

---

## Troubleshooting

### Check V4L2 modes

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
```

- D1000 should show: `1280x480 YUYV @ 60 fps`
- S1030-IR should show: `752x480 YUYV @ 60 fps`

### Verify USB device

```bash
lsusb | grep -E "1e4e:0120|04b4:4722"
```

### Disparity is all black (S1030-IR)

1. Confirm `uvc_stereo_layout:=interleaved_yuyv` is set (default in `mynteye_s1030.launch.py`).
2. Confirm `calibration_preset:=sdk_eeprom` and that the S-SDK could open the camera (check log for `EEPROM calib loaded`).
3. Check that rectified images look reasonable: `ros2 run rqt_image_view rqt_image_view /mynteye/left/image_rect`

### Points appear very far away (S1030-IR)

The node has `max_depth_m:=15.0` set in `mynteye_s1030.launch.py`. If the rectification
failed (wrong calibration), reduce to `max_depth_m:=5.0` to filter far artefacts.

### D1000 depth/image_raw is black in RViz

Use **Displays → Image → min/max** and set min=0, max=7000 (mm), or change the display
to use the `depth` rendering plugin. The raw 16UC1 values are in mm and look dark
with auto-range.

### Permission denied opening camera

```bash
ls -l /dev/video0
sudo chmod 666 /dev/video0    # temporary
# permanent: install udev rules above
```

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