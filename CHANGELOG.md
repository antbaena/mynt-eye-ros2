# Changelog

All notable changes to this project will be documented here.

## Unreleased

### Added
- `mynteye_d1000_node`: new ROS 2 node that uses the official **MYNT-EYE-D-SDK**
  (Etron eSPDI on-chip depth processor) instead of software SGBM.
  - Reads calibration (fx/fy/cx/cy/distortion/baseline) directly from the camera EEPROM.
  - Publishes `left/image_raw` (BGR8), `right/image_raw`, `depth/image_raw` (16UC1 mm),
    `left|right|depth/camera_info`, and `points` (PointCloud2, XYZ in metres).
  - Publishes static TF: `mynteye_link → mynteye_left/right` with baseline from EEPROM.
  - `stream_mode` parameter: `1280x480` (640×480/eye, default), `1280x720` (HD), `2560x720`.
  - `ir_intensity` parameter [0–10]: controls the IR projector power.
- `scripts/build_d_sdk_ubuntu24.sh`: automated build script for MYNT-EYE-D-SDK on Ubuntu 24.04.
- `launch/mynteye_d1000_hw_depth.launch.py`: dedicated launch file for the D-SDK node.
- `CMakeLists.txt`: D-SDK integration with `OPTIONAL` via `find_package(mynteyed CONFIG QUIET)`.
  If `d_install/` is absent the S-SDK node is still built; the D-SDK node is skipped with a warning.
- Updated `README.md` with Quick Start instructions for both nodes.

## 0.1.0 - Unreleased

- Initial ROS 2 Jazzy package for MYNT EYE D1000 camera (S1030-IR support planned).
- Added UVC/V4L2 backend using `1280x480 YUYV` side-by-side stereo extraction.
- Added rectified mono images, disparity, official-style `16UC1` depth, point cloud and static TF tree.
- Added BM/SGBM stereo matcher selection with defaults based on the official SDK.
- Added left-right consistency filtering, median disparity filtering and connected-component cleanup for noisy depth.
- Switched the default depth preset to balanced SGBM with mild cleanup so live depth does not collapse into a few sparse patches.
- Added legacy SDK setup helper for Ubuntu 24.04/OpenCV 4 compatibility.
- Added S1030-IR support with auto-detection of 376×480 frame size.
