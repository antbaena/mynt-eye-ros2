// Copyright 2024 MAPIR Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ROS 2 Jazzy driver for the MYNT EYE D1000 camera.
// Uses the official MYNT-EYE-D-SDK (libmynteye_depth / libeSPDI) for hardware
// depth — depth is computed on-chip at up to 60 fps with <2% error.
//
// Topics published:
//   ~/left/image_raw          sensor_msgs/Image  (BGR8, 640x480 or 1280x720)
//   ~/right/image_raw         sensor_msgs/Image  (BGR8)
//   ~/left/camera_info        sensor_msgs/CameraInfo
//   ~/right/camera_info       sensor_msgs/CameraInfo
//   ~/depth/image_raw         sensor_msgs/Image  (16UC1, values in mm)
//   ~/depth/camera_info       sensor_msgs/CameraInfo
//   ~/points                  sensor_msgs/PointCloud2 (XYZ, meters)
//
// TF published:
//   mynteye_link -> mynteye_left
//   mynteye_link -> mynteye_right

#include <atomic>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <mynteyed/camera.h>

MYNTEYE_USE_NAMESPACE

namespace mynteye_d1000 {

class MyntEyeD1000Node : public rclcpp::Node
{
public:
  MyntEyeD1000Node()
  : Node("mynteye_d1000_node")
  {
    // ---- Parameters --------------------------------------------------------
    declare_parameter<int>("dev_index",    0);
    declare_parameter<int>("framerate",    30);
    declare_parameter<int>("ir_intensity", 4);
    declare_parameter<bool>("ir_depth_only", false);
    declare_parameter<std::string>("stream_mode", "1280x480");  // or 1280x720
    declare_parameter<std::string>("base_frame_id",  "mynteye_link");
    declare_parameter<std::string>("left_frame_id",  "mynteye_left");
    declare_parameter<std::string>("right_frame_id", "mynteye_right");
    declare_parameter<std::string>("depth_frame_id", "mynteye_depth");

    dev_index_    = get_parameter("dev_index").as_int();
    framerate_    = get_parameter("framerate").as_int();
    ir_intensity_ = static_cast<uint8_t>(get_parameter("ir_intensity").as_int());
    ir_depth_only_ = get_parameter("ir_depth_only").as_bool();
    stream_mode_str_ = get_parameter("stream_mode").as_string();
    base_frame_id_  = get_parameter("base_frame_id").as_string();
    left_frame_id_  = get_parameter("left_frame_id").as_string();
    right_frame_id_ = get_parameter("right_frame_id").as_string();
    depth_frame_id_ = get_parameter("depth_frame_id").as_string();

    // ---- Publishers --------------------------------------------------------
    pub_left_   = create_publisher<sensor_msgs::msg::Image>("left/image_raw", 5);
    pub_right_  = create_publisher<sensor_msgs::msg::Image>("right/image_raw", 5);
    pub_depth_  = create_publisher<sensor_msgs::msg::Image>("depth/image_raw", 5);
    pub_left_info_  = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 5);
    pub_right_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 5);
    pub_depth_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", 5);
    pub_cloud_  = create_publisher<sensor_msgs::msg::PointCloud2>("points", 5);

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // ---- Open device and start streaming -----------------------------------
    OpenDevice();
    running_ = true;
    stream_thread_ = std::thread(&MyntEyeD1000Node::StreamLoop, this);

    RCLCPP_INFO(get_logger(), "MYNT EYE D1000 node started (hw depth via D-SDK)");
  }

  ~MyntEyeD1000Node()
  {
    running_ = false;
    if (cam_.IsOpened()) {
      cam_.Close();
    }
    if (stream_thread_.joinable()) {
      stream_thread_.join();
    }
  }

private:
  // --------------------------------------------------------------------------
  void OpenDevice()
  {
    // Enumerate devices
    std::vector<DeviceInfo> dev_infos;
    cam_.GetDeviceInfos(&dev_infos);
    if (dev_infos.empty()) {
      throw std::runtime_error(
        "No MYNT EYE device found. Check USB connection and udev rules "
        "(lsusb should show 1e4e:0120). Run: sudo udevadm trigger");
    }
    if (dev_index_ >= static_cast<int>(dev_infos.size())) {
      throw std::runtime_error("dev_index " + std::to_string(dev_index_) +
        " out of range (found " + std::to_string(dev_infos.size()) + " devices)");
    }
    RCLCPP_INFO(get_logger(), "Opening device [%d]: %s",
      dev_infos[dev_index_].index, dev_infos[dev_index_].name.c_str());

    OpenParams params(dev_infos[dev_index_].index);
    params.framerate    = framerate_;
    params.ir_intensity = ir_intensity_;
    params.ir_depth_only = ir_depth_only_;
    params.dev_mode     = DeviceMode::DEVICE_ALL;
    params.color_mode   = ColorMode::COLOR_RAW;

    if (stream_mode_str_ == "1280x720" || stream_mode_str_ == "hd") {
      params.stream_mode = StreamMode::STREAM_1280x720;
    } else if (stream_mode_str_ == "2560x720") {
      params.stream_mode = StreamMode::STREAM_2560x720;
      if (framerate_ > 30) {
        RCLCPP_WARN(get_logger(), "2560x720 mode only supports up to 30fps; clamping.");
        params.framerate = 30;
      }
    } else {
      // default: 1280x480 — stereo side-by-side, 640x480 per eye
      params.stream_mode = StreamMode::STREAM_1280x480;
    }

    params.colour_depth_value = 5000.0f;  // for colourise (unused in raw mode)

    ErrorCode err = cam_.Open(params);
    if (err != ErrorCode::SUCCESS) {
      throw std::runtime_error("Camera open failed (ErrorCode=" +
        std::to_string(static_cast<int>(err)) + ")");
    }
    if (!cam_.IsOpened()) {
      throw std::runtime_error("Camera open reported success but IsOpened() is false");
    }

    RCLCPP_INFO(get_logger(), "Camera opened. left=%s right=%s depth=%s",
      cam_.IsStreamDataEnabled(ImageType::IMAGE_LEFT_COLOR) ? "yes" : "no",
      cam_.IsStreamDataEnabled(ImageType::IMAGE_RIGHT_COLOR) ? "yes" : "no",
      cam_.IsStreamDataEnabled(ImageType::IMAGE_DEPTH) ? "yes" : "no");

    // Calibration
    bool ok = false;
    intrinsics_ = cam_.GetStreamIntrinsics(params.stream_mode, &ok);
    if (ok) {
      calib_ok_ = true;
      RCLCPP_INFO(get_logger(),
        "SDK calibration: fx=%.2f fy=%.2f cx=%.2f cy=%.2f (left %dx%d)",
        intrinsics_.left.fx, intrinsics_.left.fy,
        intrinsics_.left.cx, intrinsics_.left.cy,
        intrinsics_.left.width, intrinsics_.left.height);
    } else {
      RCLCPP_WARN(get_logger(), "SDK calibration not available, publishing approximate CameraInfo");
    }

    extrinsics_ = cam_.GetStreamExtrinsics(params.stream_mode, &ok);

    // Static TF
    PublishTF();
  }

  // --------------------------------------------------------------------------
  void StreamLoop()
  {
    bool left_ok  = cam_.IsStreamDataEnabled(ImageType::IMAGE_LEFT_COLOR);
    bool right_ok = cam_.IsStreamDataEnabled(ImageType::IMAGE_RIGHT_COLOR);
    bool depth_ok = cam_.IsStreamDataEnabled(ImageType::IMAGE_DEPTH);

    while (running_ && cam_.IsOpened()) {
      cam_.WaitForStream();

      rclcpp::Time stamp = now();

      if (left_ok) {
        auto data = cam_.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
        if (data.img) { PublishLeft(data, stamp); }
      }
      if (right_ok) {
        auto data = cam_.GetStreamData(ImageType::IMAGE_RIGHT_COLOR);
        if (data.img) { PublishRight(data, stamp); }
      }
      if (depth_ok) {
        auto data = cam_.GetStreamData(ImageType::IMAGE_DEPTH);
        if (data.img) { PublishDepth(data, stamp); }
      }
    }
    RCLCPP_INFO(get_logger(), "Stream loop exited");
  }

  // --------------------------------------------------------------------------
  void PublishLeft(const StreamData & data, const rclcpp::Time & stamp)
  {
    cv::Mat mat = data.img->To(ImageFormat::COLOR_BGR)->ToMat();
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mat).toImageMsg();
    msg->header.stamp = stamp;
    msg->header.frame_id = left_frame_id_;
    pub_left_->publish(*msg);

    auto info = BuildCameraInfo(intrinsics_.left, left_frame_id_, stamp);
    pub_left_info_->publish(info);
  }

  // --------------------------------------------------------------------------
  void PublishRight(const StreamData & data, const rclcpp::Time & stamp)
  {
    cv::Mat mat = data.img->To(ImageFormat::COLOR_BGR)->ToMat();
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mat).toImageMsg();
    msg->header.stamp = stamp;
    msg->header.frame_id = right_frame_id_;
    pub_right_->publish(*msg);

    auto info = BuildCameraInfo(intrinsics_.right, right_frame_id_, stamp);
    pub_right_info_->publish(info);
  }

  // --------------------------------------------------------------------------
  void PublishDepth(const StreamData & data, const rclcpp::Time & stamp)
  {
    // DEPTH_RAW = IMAGE_GRAY_16 = 16-bit unsigned, values in mm
    cv::Mat depth16 = data.img->To(ImageFormat::DEPTH_RAW)->ToMat();

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth16).toImageMsg();
    msg->header.stamp = stamp;
    msg->header.frame_id = depth_frame_id_;
    pub_depth_->publish(*msg);

    // CameraInfo for depth (same intrinsics as left — depth is registered to left)
    auto info = BuildCameraInfo(intrinsics_.left, depth_frame_id_, stamp);
    pub_depth_info_->publish(info);

    // Point cloud
    PublishPointCloud(depth16, stamp);
  }

  // --------------------------------------------------------------------------
  // Build PointCloud2 from 16UC1 depth (mm) using left camera intrinsics.
  // Convention matches the official D-SDK wrapper:
  //   z = depth_mm / 1000.0  (meters)
  //   x = (col - cx) * z / fx
  //   y = (row - cy) * z / fy
  //   invalid pixels: d == 0 || d == 4096
  void PublishPointCloud(const cv::Mat & depth16, const rclcpp::Time & stamp)
  {
    if (pub_cloud_->get_subscription_count() == 0) { return; }

    const double fx = calib_ok_ ? intrinsics_.left.fx : 362.2;
    const double fy = calib_ok_ ? intrinsics_.left.fy : 363.5;
    const double cx = calib_ok_ ? intrinsics_.left.cx : depth16.cols / 2.0;
    const double cy = calib_ok_ ? intrinsics_.left.cy : depth16.rows / 2.0;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = left_frame_id_;
    cloud_msg.height = 1;
    cloud_msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    std::vector<float> xs, ys, zs;
    xs.reserve(depth16.rows * depth16.cols / 4);
    ys.reserve(xs.capacity());
    zs.reserve(xs.capacity());

    for (int row = 0; row < depth16.rows; ++row) {
      const uint16_t * row_ptr = depth16.ptr<uint16_t>(row);
      for (int col = 0; col < depth16.cols; ++col) {
        uint16_t d = row_ptr[col];
        if (d == 0 || d == 4096) { continue; }
        float z = d / 1000.0f;
        float x = static_cast<float>((col - cx) * z / fx);
        float y = static_cast<float>((row - cy) * z / fy);
        xs.push_back(x);
        ys.push_back(y);
        zs.push_back(z);
      }
    }

    cloud_msg.width = static_cast<uint32_t>(xs.size());
    modifier.resize(cloud_msg.width);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    for (size_t i = 0; i < xs.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = xs[i];
      *iter_y = ys[i];
      *iter_z = zs[i];
    }

    pub_cloud_->publish(cloud_msg);
  }

  // --------------------------------------------------------------------------
  sensor_msgs::msg::CameraInfo BuildCameraInfo(
    const CameraIntrinsics & in,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp    = stamp;
    info.header.frame_id = frame_id;

    if (calib_ok_) {
      info.width  = static_cast<uint32_t>(in.width);
      info.height = static_cast<uint32_t>(in.height);
      info.distortion_model = "plumb_bob";
      info.d.assign(in.coeffs, in.coeffs + 5);
      // K = [fx 0 cx; 0 fy cy; 0 0 1]
      info.k = {in.fx, 0.0, in.cx,
                0.0, in.fy, in.cy,
                0.0,  0.0,  1.0};
      // R = rectification rotation (3x3)
      for (int i = 0; i < 9; ++i) { info.r[i] = in.r[i]; }
      // P = 3x4 projection matrix
      for (int i = 0; i < 12; ++i) { info.p[i] = in.p[i]; }
    } else {
      // Approximate for 640x480
      info.width  = 640;
      info.height = 480;
      info.distortion_model = "plumb_bob";
      info.d = {0, 0, 0, 0, 0};
      info.k = {362.2, 0.0, 320.0,
                0.0, 363.5, 240.0,
                0.0,   0.0,   1.0};
      info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
      info.p = {362.2, 0.0, 320.0, 0.0,
                0.0, 363.5, 240.0, 0.0,
                0.0,   0.0,   1.0, 0.0};
    }
    return info;
  }

  // --------------------------------------------------------------------------
  void PublishTF()
  {
    // Extract baseline from extrinsics (translation[0] is typically -baseline_m).
    // Some D-SDK versions return translation in mm (e.g. -120.0) instead of meters.
    double baseline = 0.12002;  // 120 mm default
    if (calib_ok_) {
      double raw = std::abs(extrinsics_.translation[0]);
      // Heuristic: if |translation| > 1.0 the unit is mm, convert to meters.
      if (raw > 1.0) {
        RCLCPP_INFO(get_logger(),
          "SDK extrinsics translation looks like mm (%.1f); converting to meters.", raw);
        raw /= 1000.0;
      }
      if (raw < 0.05 || raw > 0.3) {
        RCLCPP_WARN(get_logger(),
          "SDK baseline out of range (%.4f m), falling back to 0.12002 m", raw);
      } else {
        baseline = raw;
      }
    }

    const rclcpp::Time now_t = now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    // base -> left (optical frame at camera origin)
    geometry_msgs::msg::TransformStamped t_left;
    t_left.header.stamp    = now_t;
    t_left.header.frame_id = base_frame_id_;
    t_left.child_frame_id  = left_frame_id_;
    t_left.transform.translation.x = 0.0;
    t_left.transform.translation.y = 0.0;
    t_left.transform.translation.z = 0.0;
    t_left.transform.rotation.x = 0.0;
    t_left.transform.rotation.y = 0.0;
    t_left.transform.rotation.z = 0.0;
    t_left.transform.rotation.w = 1.0;
    transforms.push_back(t_left);

    // base -> right (offset by baseline along -x)
    geometry_msgs::msg::TransformStamped t_right;
    t_right.header.stamp    = now_t;
    t_right.header.frame_id = base_frame_id_;
    t_right.child_frame_id  = right_frame_id_;
    t_right.transform.translation.x = -baseline;
    t_right.transform.translation.y = 0.0;
    t_right.transform.translation.z = 0.0;
    t_right.transform.rotation.x = 0.0;
    t_right.transform.rotation.y = 0.0;
    t_right.transform.rotation.z = 0.0;
    t_right.transform.rotation.w = 1.0;
    transforms.push_back(t_right);

    tf_broadcaster_->sendTransform(transforms);
    RCLCPP_INFO(get_logger(),
      "Published static TF: %s -> {%s, %s} (baseline=%.4f m)",
      base_frame_id_.c_str(), left_frame_id_.c_str(), right_frame_id_.c_str(), baseline);
  }

  // --------------------------------------------------------------------------
  // Member variables
  Camera cam_;
  std::thread stream_thread_;
  std::atomic<bool> running_{false};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      pub_right_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_info_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  int         dev_index_;
  int         framerate_;
  uint8_t     ir_intensity_;
  bool        ir_depth_only_;
  std::string stream_mode_str_;
  std::string base_frame_id_;
  std::string left_frame_id_;
  std::string right_frame_id_;
  std::string depth_frame_id_;

  StreamIntrinsics  intrinsics_{};
  StreamExtrinsics  extrinsics_{};
  bool              calib_ok_{false};
};

}  // namespace mynteye_d1000

// =============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<mynteye_d1000::MyntEyeD1000Node>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mynteye_d1000_node"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
