# Changelog

All notable changes to this project will be documented here.

## 0.1.0 - Unreleased

- Initial ROS 2 Jazzy package for MYNT EYE S1030/D1000 cameras.
- Added UVC/V4L2 backend using `1280x480 YUYV` side-by-side stereo extraction.
- Added rectified mono images, disparity, official-style `16UC1` depth, point cloud and static TF tree.
- Added BM/SGBM stereo matcher selection with defaults based on the official SDK.
- Added left-right consistency filtering, median disparity filtering and connected-component cleanup for noisy depth.
- Switched the default depth preset to balanced SGBM with mild cleanup so live depth does not collapse into a few sparse patches.
- Added legacy SDK setup helper for Ubuntu 24.04/OpenCV 4 compatibility.
