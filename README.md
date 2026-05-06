# MYNT EYE D1000 ROS 2 Jazzy

ROS 2 Jazzy driver/wrapper for the MYNT EYE D1000 stereo IR depth camera on Ubuntu 24.04. S1030-IR support is planned.

This project keeps the old MYNT-EYE-S-SDK usable on modern systems, then exposes the camera through a native ROS 2 node with stereo images, rectified images, disparity, depth, point cloud, TF, and optional SDK IMU support.

## Status

Beta hardware driver. The tested camera enumerates as:

- USB camera: `1e4e:0120`
- V4L2 name: `MYNT EYE: MYNT-EYE-D1000`
- Working stereo mode: `1280x480 YUYV`, split into two `640x480` mono IR images

The default ROS 2 backend uses V4L2/UVC because the legacy SDK streaming path is fragile on Ubuntu 24.04. The SDK is still used as a dependency and reference implementation.

## Repository Layout

```text
mynteye_ws/src/mynteye_camera/  ROS 2 package
scripts/                         setup helpers for the legacy SDK
.github/                         CI and issue templates
```

Do not commit SDK source/build trees or colcon artifacts. They are ignored by `.gitignore` and should be generated locally.

## Quick Start

Install ROS 2 Jazzy and system dependencies:

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake pkg-config \
  ros-jazzy-desktop ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-tf2 ros-jazzy-tf2-ros \
  libopencv-dev libboost-filesystem-dev libpcl-dev v4l-utils
```

Optional but recommended: install the included udev rules so the camera can be
opened without running ROS 2 as root:

```bash
./scripts/install_udev.sh
```

Build the patched legacy SDK into `./install`:

```bash
./scripts/build_legacy_sdk_ubuntu24.sh
```

Build the ROS 2 package:

```bash
cd mynteye_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mynteye_camera --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

Launch the camera:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mynteye_camera mynteye.launch.py device_wait_sec:=60
```

## Documentation

See [mynteye_ws/src/mynteye_camera/README.md](mynteye_ws/src/mynteye_camera/README.md) for topics, launch arguments, depth tuning, TF frames, and troubleshooting.

Useful depth notes are in [mynteye_ws/src/mynteye_camera/docs/DEPTH_TUNING.md](mynteye_ws/src/mynteye_camera/docs/DEPTH_TUNING.md).

## License

Apache-2.0. See [mynteye_ws/src/mynteye_camera/LICENSE](mynteye_ws/src/mynteye_camera/LICENSE).
