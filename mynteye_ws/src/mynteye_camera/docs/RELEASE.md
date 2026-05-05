# Release Notes And Branching

Recommended branch layout for the public repository:

- `main`: stable ROS 2 Jazzy driver, always buildable.
- `develop`: integration branch for hardware-tested changes.
- `feature/<short-name>`: focused feature branches.
- `fix/<short-name>`: focused bug-fix branches.
- `release/vX.Y.Z`: release preparation branches when needed.

Use semantic versions in `package.xml` and Git tags:

```bash
git tag -a v0.1.0 -m "mynteye_camera v0.1.0"
git push origin v0.1.0
```

Before tagging a release:

1. Build the SDK helper from a clean checkout.
2. Run `colcon build --symlink-install --packages-select mynteye_camera`.
3. Launch the camera on Ubuntu 24.04 + ROS 2 Jazzy.
4. Confirm stereo images are two different eyes in `1280x480 YUYV` mode.
5. Confirm `/mynteye/debug/disparity`, `/mynteye/left/depth/image_raw`, and `/mynteye/points` publish.
6. Update `CHANGELOG.md`.
