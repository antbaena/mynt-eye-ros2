# Depth Tuning

The MYNT EYE S1030 is an active IR stereo camera. Its official SDK does not try to publish a dense depth map everywhere: it computes disparity, rejects weak matches, and publishes invalid depth as zero in a `16UC1` image. This ROS 2 node publishes `16UC1` depth by default too, but uses a balanced SGBM preset so the live RViz view does not collapse into only a few valid patches.

## Defaults Chosen From The Official SDK

The official ROS 1 wrapper defaults to `disparity_computing_method: 1`, which is BM. The SDK BM defaults are:

- `block_size`: 15
- `num_disparities`: 128
- `pre_filter_size`: 9
- `pre_filter_cap`: 31
- `uniqueness_ratio`: 60
- `texture_threshold`: 10
- `speckle_window_size`: 100
- `speckle_range`: 4

The node can map those stricter SDK BM defaults through:

```bash
ros2 launch mynteye_camera mynteye.launch.py stereo_matcher:=bm
```

`block_size:=0`, `uniqueness_ratio:=0`, `speckle_range:=0`, and `pre_filter_cap:=0` mean "use the official default for the selected matcher".

## Why Depth Can Look Noisy

Stereo depth is only reliable where the left and right cameras see the same textured surface. Noise usually comes from:

- using `640x480` UVC mode, which is not stereo on the tested camera;
- swapped left/right images;
- low-texture walls, reflections, glass, or saturated IR;
- missing/approximate calibration;
- displaying every marginal disparity as dense `32FC1` depth.

The tested stereo mode is `1280x480 YUYV`. The node extracts two `640x480` mono images from the luminance halves. Covering either lens should strongly degrade depth and point cloud output.

## Balanced Default Preset

The launch defaults favor useful coverage with moderate filtering:

```bash
ros2 launch mynteye_camera mynteye.launch.py \
  stereo_matcher:=sgbm \
  depth_image_encoding:=16UC1 \
  enable_lr_check:=false \
  lr_check_tolerance_px:=3.0 \
  disparity_median_kernel:=3 \
  depth_cleanup_kernel:=0 \
  depth_min_component_area:=20
```

This keeps most valid stereo regions, removes only tiny islands, and publishes invalid depth as `0` millimeters.

## If Depth Is Too Sparse

Relax filtering further:

```bash
ros2 launch mynteye_camera mynteye.launch.py \
  stereo_matcher:=sgbm \
  depth_cleanup_kernel:=0 \
  depth_min_component_area:=0 \
  enable_lr_check:=false \
  disparity_median_kernel:=3
```

If you are using BM and only see a few patches, switch back to SGBM:

```bash
ros2 launch mynteye_camera mynteye.launch.py \
  stereo_matcher:=sgbm
```

SGBM tends to fill more pixels but can show more bad matches in noisy IR scenes.

## If Depth Is Still Noisy

Tighten filtering:

```bash
ros2 launch mynteye_camera mynteye.launch.py \
  enable_lr_check:=true \
  lr_check_tolerance_px:=1.0 \
  depth_cleanup_kernel:=3 \
  depth_min_component_area:=120 \
  max_depth_m:=5.0
```

For the stricter SDK-like BM preset:

```bash
ros2 launch mynteye_camera mynteye.launch.py \
  stereo_matcher:=bm \
  enable_lr_check:=true \
  depth_cleanup_kernel:=3 \
  depth_min_component_area:=120 \
  lr_check_tolerance_px:=1.0 \
  bm_texture_threshold:=20 \
  max_depth_m:=5.0
```

For measurement, perform camera-specific stereo calibration and pass calibrated `fx`, `fy`, `cx`, `cy`, and `baseline_m`. The bundled `sdk_default` preset is useful for visualization, but it is not a substitute for calibration of your exact unit.

## Debug Checklist

1. Confirm V4L2 mode:

   ```bash
   v4l2-ctl --list-formats-ext -d /dev/video0
   ```

2. Confirm the node opened `1280x480`, not `640x480`.

3. Compare `/mynteye/left/image_rect` and `/mynteye/right/image_rect`.

4. View `/mynteye/debug/disparity` before looking at depth.

5. Check `/mynteye/points` in RViz with fixed frame `mynteye_link`.
