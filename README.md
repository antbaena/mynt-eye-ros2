# MYNT EYE D1000 ROS 2 Jazzy

ROS 2 Jazzy driver/wrapper for the MYNT EYE D1000 stereo depth camera on Ubuntu 24.04.
S1030-IR support is also included.

This project provides **two complementary ROS 2 nodes**:

| Node | Backend | Depth | Cameras |
|------|---------|-------|---------|
| `mynteye_camera_node` | V4L2/UVC + software SGBM | computed on CPU | D1000, S1030-IR |
| `mynteye_d1000_node` | MYNT-EYE-D-SDK + on-chip eSPDI | **hardware depth** | D1000 only |

The `mynteye_d1000_node` reads calibration from the camera's EEPROM and delivers
hardware-accelerated depth at up to 60 fps (<2 % error, 0.32 – 7 m working range).

## Status

Beta hardware driver. The tested camera enumerates as:

- USB camera: `1e4e:0120`
- V4L2 name: `MYNT EYE: MYNT-EYE-D1000`
- Working stereo mode: `1280x480 YUYV`, split into two `640x480` images

## Repository Layout

```text
mynteye_ws/src/mynteye_camera/  ROS 2 package (both nodes)
scripts/                         SDK build helpers
  build_legacy_sdk_ubuntu24.sh  — builds MYNT-EYE-S-SDK into ./install/
  build_d_sdk_ubuntu24.sh       — builds MYNT-EYE-D-SDK into ./d_install/
.github/                         CI and issue templates
```

Do not commit SDK source/build trees or colcon artifacts. They are ignored by `.gitignore` and should be generated locally.

## Quick Start — Hardware Depth (mynteye_d1000_node)

```bash
# 1. Install system dependencies
sudo apt update && sudo apt install -y \
  git build-essential cmake pkg-config \
  ros-jazzy-desktop ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-tf2 ros-jazzy-tf2-ros \
  libopencv-dev libusb-dev libusb-1.0-0-dev

# 2. Build both SDKs
./scripts/build_legacy_sdk_ubuntu24.sh   # S-SDK  -> ./install/
./scripts/build_d_sdk_ubuntu24.sh        # D-SDK  -> ./d_install/

# 3. Build the ROS 2 package
cd mynteye_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mynteye_camera \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# 4. Launch with hardware depth
source install/setup.bash
ros2 launch mynteye_camera mynteye_d1000_hw_depth.launch.py
```

## Quick Start — Software Depth (mynteye_camera_node)

```bash
# 1. Install system dependencies (same as above)
# 2. Build only the S-SDK
./scripts/build_legacy_sdk_ubuntu24.sh

# 3. Build and launch
cd mynteye_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mynteye_camera \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch mynteye_camera mynteye.launch.py device_wait_sec:=60
```

## Documentation

See [mynteye_ws/src/mynteye_camera/README.md](mynteye_ws/src/mynteye_camera/README.md) for topics, launch arguments, depth tuning, TF frames, and troubleshooting.

Useful depth notes are in [mynteye_ws/src/mynteye_camera/docs/DEPTH_TUNING.md](mynteye_ws/src/mynteye_camera/docs/DEPTH_TUNING.md).

## License

Apache-2.0. See [mynteye_ws/src/mynteye_camera/LICENSE](mynteye_ws/src/mynteye_camera/LICENSE).
