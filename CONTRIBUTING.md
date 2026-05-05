# Contributing

Thanks for helping keep this legacy camera useful on ROS 2.

## Development Setup

1. Use Ubuntu 24.04 with ROS 2 Jazzy.
2. Run `./scripts/build_legacy_sdk_ubuntu24.sh` from the repository root.
3. Build the package with:

   ```bash
   cd mynteye_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --packages-select mynteye_camera --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
   ```

## Pull Requests

- Keep changes focused and hardware-test camera behavior when possible.
- Do not commit `build/`, `install/`, `log/`, SDK clones, ROS bags or local editor settings.
- Document new launch parameters in `mynteye_ws/src/mynteye_camera/README.md`.
- For depth changes, include the tested matcher, lighting, V4L2 mode and whether `stereo_matcher:=bm` or `sgbm` was used.

## Branching

Use `main` for stable Jazzy code and short-lived `feature/<name>` or `fix/<name>` branches for changes. See `mynteye_ws/src/mynteye_camera/docs/RELEASE.md`.
