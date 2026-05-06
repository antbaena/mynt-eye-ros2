# MYNT EYE ROS 2 Jazzy Driver

ROS 2 Jazzy driver for the **MYNT EYE D1000** and **MYNT EYE S1030-IR** stereo cameras on Ubuntu 24.04.

This project provides **two complementary ROS 2 nodes**:

| Node | Camera | Backend | Depth method |
|------|--------|---------|--------------|
| `mynteye_d1000_node` | **D1000** | MYNT-EYE-D-SDK (libeSPDI) | **Hardware eSPDI on-chip** (0.32–7 m, <2 % error, up to 60 fps) |
| `mynteye_camera_node` | **D1000 / S1030-IR** | V4L2 UVC + MYNT-EYE-S-SDK | Software SGBM on CPU (wider range, configurable) |

> **Recommended for D1000:** `mynteye_d1000_node` — hardware depth is always better.
> **Recommended for S1030-IR:** `mynteye_camera_node` with `calibration_preset:=sdk_eeprom`.

---

## Tested hardware

| Camera | USB ID | V4L2 name | Native stereo mode |
|--------|--------|-----------|--------------------|
| MYNT EYE D1000 | `1e4e:0120` | `MYNT EYE: MYNT-EYE-D1000` | `1280×480 YUYV` (side-by-side → 2 × 640×480) |
| MYNT EYE S1030-IR | `04b4:4722` | `MYNT-EYE-S1030` | `752×480 YUYV` (interleaved → 2 × 752×480) |

---

## Repository layout

```text
mynteye_ws/src/mynteye_camera/   ROS 2 package (both nodes)
  launch/
    mynteye_d1000_hw_depth.launch.py  — D1000 hardware depth (mynteye_d1000_node)
    mynteye_s1030.launch.py           — S1030-IR SGBM depth (mynteye_camera_node)
    mynteye.launch.py                 — generic UVC launch (D1000 or S1030-IR)
scripts/
  build_legacy_sdk_ubuntu24.sh   — builds MYNT-EYE-S-SDK → ./install/
  build_d_sdk_ubuntu24.sh        — builds MYNT-EYE-D-SDK → ./d_install/
  read_s_sdk_calib.cpp           — standalone tool: print EEPROM calibration
```

SDK source/build trees and colcon artifacts are listed in `.gitignore` and must be generated locally.

---

## Prerequisites

```bash
sudo apt update && sudo apt install -y \
  git build-essential cmake pkg-config \
  ros-jazzy-desktop ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-tf2 ros-jazzy-tf2-ros \
  libopencv-dev libusb-dev libusb-1.0-0-dev
```

### udev rules (required for both cameras)

```bash
# D1000 (1e4e:0120)
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1e4e", ATTR{idProduct}=="0120", MODE="0666", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-mynteye-d1000.rules

# S1030-IR (04b4:4722)
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04b4", ATTR{idProduct}=="4722", MODE="0666", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-mynteye-s1030.rules

sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG plugdev $USER   # re-login after this
```

---

## Installation

### 1. Build the MYNT-EYE-S-SDK (needed by both nodes)

```bash
cd /path/to/mynt-eye-ros2
./scripts/build_legacy_sdk_ubuntu24.sh
# Installs to ./install/
```

### 2. Build the MYNT-EYE-D-SDK (needed only by `mynteye_d1000_node`)

```bash
./scripts/build_d_sdk_ubuntu24.sh
# Installs to ./d_install/
```

### 3. Build the ROS 2 package

```bash
cd mynteye_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mynteye_camera \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

---

## Quick Start — D1000 (hardware depth)

```bash
source /opt/ros/jazzy/setup.bash
source mynteye_ws/install/setup.bash

ros2 launch mynteye_camera mynteye_d1000_hw_depth.launch.py
```

Default stream mode: `1280×480 YUYV` (side-by-side stereo, 640×480 per eye, 30 fps).
Depth is computed on-chip by the eSPDI processor. Working range: **0.32 – 7 m**.

Key topics:

| Topic | Type | Description |
|-------|------|-------------|
| `left/image_raw` | `Image` (BGR8) | Left camera image |
| `right/image_raw` | `Image` (BGR8) | Right camera image |
| `depth/image_raw` | `Image` (16UC1, mm) | Hardware depth map |
| `points` | `PointCloud2` | XYZ point cloud (meters) |
| `left/camera_info` | `CameraInfo` | EEPROM calibration |

**Optional parameters:**

```bash
ros2 launch mynteye_camera mynteye_d1000_hw_depth.launch.py \
  stream_mode:=1280x480 \   # 640x480x480 or 1280x480 or 1280x720 or 2560x720
  framerate:=30 \           # max 60 (30 for 2560x720)
  ir_intensity:=4 \         # 0-10
  ir_depth_only:=false      # true = depth only (no color stream)
```

---

## Quick Start — S1030-IR (software SGBM depth)

```bash
source /opt/ros/jazzy/setup.bash
source mynteye_ws/install/setup.bash

ros2 launch mynteye_camera mynteye_s1030.launch.py
```

On first launch the node reads calibration from the camera EEPROM via S-SDK (`calibration_preset:=sdk_eeprom`).
Stream: `752×480 YUYV` interleaved → 752×480 per eye, 60 fps.
Depth range: **0.3 – 15 m** (software SGBM).

Key topics:

| Topic | Type | Description |
|-------|------|-------------|
| `mynteye/left/image_raw` | `Image` (mono8) | Left IR image |
| `mynteye/right/image_raw` | `Image` (mono8) | Right IR image |
| `mynteye/left/image_rect` | `Image` (mono8) | Rectified left |
| `mynteye/left/depth/image_raw` | `Image` (16UC1, mm) | SGBM depth |
| `mynteye/points` | `PointCloud2` | XYZ + intensity |
| `mynteye/debug/disparity` | `Image` (mono8) | Disparity preview |

**Optional parameters:**

```bash
ros2 launch mynteye_camera mynteye_s1030.launch.py \
  calibration_preset:=sdk_eeprom \   # sdk_eeprom (default) or sdk_s1030_default
  stereo_matcher:=sgbm \             # sgbm or bm
  num_disparities:=128 \
  block_size:=9 \
  min_depth_m:=0.3 \
  max_depth_m:=15.0
```

### Read EEPROM calibration manually

```bash
cd /path/to/mynt-eye-ros2
g++ -std=c++17 scripts/read_s_sdk_calib.cpp \
    -I install/include $(pkg-config --cflags --libs opencv4) \
    -L install/lib -lmynteye -Wl,-rpath,install/lib \
    -o /tmp/read_s_sdk_calib
LD_LIBRARY_PATH=install/lib /tmp/read_s_sdk_calib
```

---

## D1000 stream modes (D-SDK `StreamMode`)

| Mode | Resolution | Max fps | Right color | Notes |
|------|-----------|---------|-------------|-------|
| `STREAM_640x480` | 640×480 | 60 | — | Left only |
| `STREAM_1280x480` | 1280×480 | 60 | yes | Side-by-side stereo (default) |
| `STREAM_1280x720` | 1280×720 | 60 | — | Left only, HD |
| `STREAM_2560x720` | 2560×720 | **30** | yes | Side-by-side HD stereo |

Use `stream_mode:=1280x480` (default) for full stereo + depth.
Use `stream_mode:=2560x720` for HD stereo (max 30 fps).

---

## TF frames

Both nodes publish static transforms on `/tf_static`:

```
mynteye_link
  ├── mynteye_left    (stereo center → left camera)
  ├── mynteye_right   (stereo center → right camera)
  ├── mynteye_points  (= mynteye_left for point cloud)
  └── mynteye_imu     (only mynteye_camera_node)
```

Set `base_at_stereo_center:=false` to place `mynteye_link` at the left camera origin.

---

## Documentation

Full parameter reference, depth tuning and troubleshooting:
[mynteye_ws/src/mynteye_camera/README.md](mynteye_ws/src/mynteye_camera/README.md)

Depth tuning guide:
[mynteye_ws/src/mynteye_camera/docs/DEPTH_TUNING.md](mynteye_ws/src/mynteye_camera/docs/DEPTH_TUNING.md)

---

## License

Apache-2.0 — see [mynteye_ws/src/mynteye_camera/LICENSE](mynteye_ws/src/mynteye_camera/LICENSE).
