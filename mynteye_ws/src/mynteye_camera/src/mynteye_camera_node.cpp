// Copyright 2026
// ROS 2 Jazzy wrapper for the MYNT EYE D1000 camera.
// S1030-IR support is planned (likely same PID; unconfirmed).
//
// Subscribes (via SDK callbacks) to:
//   - LEFT  / RIGHT mono images
//   - IMU (gyro + accel)
//
// Publishes:
//   - /mynteye/left/image_raw   sensor_msgs/Image   (mono8)
//   - /mynteye/right/image_raw  sensor_msgs/Image   (mono8)
//   - /mynteye/imu/data_raw     sensor_msgs/Imu
//   - stereo disparity/depth images and an XYZ+intensity point cloud

#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cv_bridge/cv_bridge.hpp>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"

using namespace std::chrono_literals;
MYNTEYE_USE_NAMESPACE

namespace mynteye_camera
{

class MyntEyeNode : public rclcpp::Node {
public:
  MyntEyeNode()
  : Node("mynteye_camera_node")
  {
    // Parameters
    backend_ = declare_parameter<std::string>("backend", "uvc");
    video_device_ = declare_parameter<std::string>("video_device", "");
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 480);
    fps_ = declare_parameter<int>("fps", 25);
    device_wait_sec_ = declare_parameter<int>("device_wait_sec", 15);
    split_stereo_ = declare_parameter<bool>("split_stereo", true);
    uvc_stereo_layout_ = declare_parameter<std::string>("uvc_stereo_layout", "auto");
    swap_stereo_ = declare_parameter<bool>("swap_stereo", false);
    enable_depth_ = declare_parameter<bool>("enable_depth", true);
    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", true);
    publish_debug_images_ = declare_parameter<bool>("publish_debug_images", true);
    publish_rectified_images_ = declare_parameter<bool>("publish_rectified_images", true);
    rectify_stereo_ = declare_parameter<bool>("rectify_stereo", true);
    calibration_preset_ = declare_parameter<std::string>("calibration_preset", "sdk_default");
    stereo_preprocess_ = declare_parameter<bool>("stereo_preprocess", true);
    clahe_clip_limit_ = declare_parameter<double>("clahe_clip_limit", 2.0);
    clahe_tile_grid_size_ = declare_parameter<int>("clahe_tile_grid_size", 8);
    baseline_m_ = declare_parameter<double>("baseline_m", 0.12);
    fx_ = declare_parameter<double>("fx", 0.0);
    fy_ = declare_parameter<double>("fy", 0.0);
    cx_ = declare_parameter<double>("cx", 0.0);
    cy_ = declare_parameter<double>("cy", 0.0);
    horizontal_fov_deg_ = declare_parameter<double>("horizontal_fov_deg", 90.0);
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.20);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 10.0);
    stereo_matcher_type_ = declare_parameter<std::string>("stereo_matcher", "sgbm");
    min_disparity_ = declare_parameter<int>("min_disparity", 0);
    num_disparities_ = declare_parameter<int>("num_disparities", 128);
    block_size_ = declare_parameter<int>("block_size", 0);
    uniqueness_ratio_ = declare_parameter<int>("uniqueness_ratio", 0);
    speckle_window_size_ = declare_parameter<int>("speckle_window_size", 100);
    speckle_range_ = declare_parameter<int>("speckle_range", 0);
    disp12_max_diff_ = declare_parameter<int>("disp12_max_diff", 1);
    pre_filter_cap_ = declare_parameter<int>("pre_filter_cap", 0);
    bm_pre_filter_size_ = declare_parameter<int>("bm_pre_filter_size", 9);
    bm_texture_threshold_ = declare_parameter<int>("bm_texture_threshold", 10);
    disparity_median_kernel_ = declare_parameter<int>("disparity_median_kernel", 3);
    depth_cleanup_kernel_ = declare_parameter<int>("depth_cleanup_kernel", 0);
    depth_min_component_area_ = declare_parameter<int>("depth_min_component_area", 20);
    depth_image_encoding_ = declare_parameter<std::string>("depth_image_encoding", "16UC1");
    enable_lr_check_ = declare_parameter<bool>("enable_lr_check", false);
    lr_check_tolerance_px_ = declare_parameter<double>("lr_check_tolerance_px", 3.0);
    pointcloud_stride_ = declare_parameter<int>("pointcloud_stride", 2);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    base_frame_ = declare_parameter<std::string>("base_frame", "mynteye_link");
    base_at_stereo_center_ = declare_parameter<bool>("base_at_stereo_center", true);
    pointcloud_ros_coordinates_ = declare_parameter<bool>("pointcloud_ros_coordinates", true);
    imu_x_m_ = declare_parameter<double>("imu_x_m", 0.0);
    imu_y_m_ = declare_parameter<double>("imu_y_m", 0.0);
    imu_z_m_ = declare_parameter<double>("imu_z_m", 0.0);
    frame_id_left_ = declare_parameter<std::string>("left_frame", "mynteye_left");
    frame_id_right_ = declare_parameter<std::string>("right_frame", "mynteye_right");
    frame_id_points_ = declare_parameter<std::string>("points_frame", "mynteye_points");
    frame_id_imu_ = declare_parameter<std::string>("imu_frame", "mynteye_imu");

    consecutive_uvc_failures_ = 0;
    ConfigureStereoMatcher();
    PublishStaticTfTree();

    // QoS recommended for sensor streams: best-effort, depth 5
    auto img_qos = rclcpp::SensorDataQoS().keep_last(5);
    auto imu_qos = rclcpp::SensorDataQoS().keep_last(50);
    auto pointcloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    pub_left_ = create_publisher<sensor_msgs::msg::Image>(
        "mynteye/left/image_raw", img_qos);
    pub_right_ = create_publisher<sensor_msgs::msg::Image>(
        "mynteye/right/image_raw", img_qos);
    pub_left_rect_ = create_publisher<sensor_msgs::msg::Image>(
      "mynteye/left/image_rect", img_qos);
    pub_right_rect_ = create_publisher<sensor_msgs::msg::Image>(
      "mynteye/right/image_rect", img_qos);
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(
        "mynteye/imu/data_raw", imu_qos);
    pub_left_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "mynteye/left/camera_info", img_qos);
    pub_right_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "mynteye/right/camera_info", img_qos);
    pub_left_disparity_ = create_publisher<sensor_msgs::msg::Image>(
      "mynteye/left/disparity/image_raw", img_qos);
    pub_right_disparity_ = create_publisher<sensor_msgs::msg::Image>(
      "mynteye/right/disparity/image_raw", img_qos);
    pub_left_depth_ = create_publisher<sensor_msgs::msg::Image>(
      "mynteye/left/depth/image_raw", img_qos);
    pub_right_depth_ = create_publisher<sensor_msgs::msg::Image>(
      "mynteye/right/depth/image_raw", img_qos);
    pub_debug_disparity_ = create_publisher<sensor_msgs::msg::Image>(
      "mynteye/debug/disparity", img_qos);
    pub_points_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "mynteye/points", pointcloud_qos);

    if (!OpenCamera()) {
      RCLCPP_FATAL(get_logger(),
                   "Failed to open MYNT EYE camera. Is it plugged in and udev configured?");
      throw std::runtime_error("MYNT EYE open failed");
    }
  }

  ~MyntEyeNode() override
  {
    if (api_) {
      api_->Stop(Source::ALL);
    }
    if (uvc_capture_.isOpened()) {
      uvc_capture_.release();
    }
  }

private:
  static int MakeOddAtLeast(int value, int minimum)
  {
    value = std::max(value, minimum);
    return (value % 2 == 0) ? value + 1 : value;
  }

  static int MakeMultipleOf16(int value)
  {
    value = std::max(value, 16);
    return ((value + 15) / 16) * 16;
  }

  void ConfigureStereoMatcher()
  {
    std::transform(stereo_matcher_type_.begin(), stereo_matcher_type_.end(),
                   stereo_matcher_type_.begin(), [](unsigned char c) {return std::tolower(c);});
    if (stereo_matcher_type_ != "bm" && stereo_matcher_type_ != "sgbm") {
      RCLCPP_WARN(get_logger(), "Unknown stereo_matcher '%s', using sgbm",
          stereo_matcher_type_.c_str());
      stereo_matcher_type_ = "sgbm";
    }

    const bool use_bm = stereo_matcher_type_ == "bm";
    if (block_size_ <= 0) {
      block_size_ = use_bm ? 15 : 3;
    }
    block_size_ = MakeOddAtLeast(block_size_, 3);
    num_disparities_ = MakeMultipleOf16(num_disparities_);
    const int pre_filter_cap = pre_filter_cap_ > 0 ? pre_filter_cap_ : (use_bm ? 31 : 63);
    const int uniqueness_ratio = uniqueness_ratio_ > 0 ? uniqueness_ratio_ : (use_bm ? 60 : 10);
    const int speckle_range = speckle_range_ > 0 ? speckle_range_ : (use_bm ? 4 : 32);

    if (use_bm) {
      stereo_matcher_ = cv::StereoBM::create(num_disparities_, block_size_);
      right_stereo_matcher_ = cv::StereoBM::create(num_disparities_, block_size_);
      auto left_bm = stereo_matcher_.dynamicCast<cv::StereoBM>();
      auto right_bm = right_stereo_matcher_.dynamicCast<cv::StereoBM>();
      for (const auto & matcher : {left_bm, right_bm}) {
        matcher->setMinDisparity(min_disparity_);
        matcher->setPreFilterCap(std::max(1, pre_filter_cap));
        matcher->setUniquenessRatio(std::max(0, uniqueness_ratio));
        matcher->setSpeckleWindowSize(std::max(0, speckle_window_size_));
        matcher->setSpeckleRange(std::max(0, speckle_range));
        matcher->setDisp12MaxDiff(disp12_max_diff_);
        matcher->setPreFilterSize(MakeOddAtLeast(bm_pre_filter_size_, 5));
        matcher->setTextureThreshold(std::max(0, bm_texture_threshold_));
        matcher->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
      }
      right_stereo_matcher_->setMinDisparity(-(min_disparity_ + num_disparities_));
    } else {
      stereo_matcher_ = cv::StereoSGBM::create(
          min_disparity_, num_disparities_, block_size_);
      right_stereo_matcher_ = cv::StereoSGBM::create(
          -(min_disparity_ + num_disparities_), num_disparities_, block_size_);
      const int p1 = 8 * block_size_ * block_size_;
      const int p2 = 32 * block_size_ * block_size_;
      auto left_sgbm = stereo_matcher_.dynamicCast<cv::StereoSGBM>();
      auto right_sgbm = right_stereo_matcher_.dynamicCast<cv::StereoSGBM>();
      for (const auto & matcher : {left_sgbm, right_sgbm}) {
        matcher->setP1(p1);
        matcher->setP2(p2);
        matcher->setPreFilterCap(std::max(1, pre_filter_cap));
        matcher->setUniquenessRatio(std::max(0, uniqueness_ratio));
        matcher->setSpeckleWindowSize(std::max(0, speckle_window_size_));
        matcher->setSpeckleRange(std::max(0, speckle_range));
        matcher->setDisp12MaxDiff(disp12_max_diff_);
        matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
      }
    }

    RCLCPP_INFO(get_logger(),
          "Depth utilities: %s, matcher %s, baseline %.3f m, num disparities %d, block size %d, calibration %s",
          enable_depth_ ? "enabled" : "disabled", stereo_matcher_type_.c_str(), baseline_m_,
          num_disparities_, block_size_, calibration_preset_.c_str());
  }

  double FocalXFor(int image_width) const
  {
    if (fx_ > 0.0) {
      return fx_;
    }
    const double fov_rad = std::max(1.0, std::min(179.0, horizontal_fov_deg_)) * M_PI / 180.0;
    return (static_cast<double>(image_width) * 0.5) / std::tan(fov_rad * 0.5);
  }

  double FocalYFor(int image_width) const
  {
    return (fy_ > 0.0) ? fy_ : FocalXFor(image_width);
  }

  double CenterXFor(int image_width) const
  {
    return (cx_ > 0.0) ? cx_ : (static_cast<double>(image_width) - 1.0) * 0.5;
  }

  double CenterYFor(int image_height) const
  {
    return (cy_ > 0.0) ? cy_ : (static_cast<double>(image_height) - 1.0) * 0.5;
  }

  sensor_msgs::msg::CameraInfo BuildCameraInfo(
    const std_msgs::msg::Header & header, int image_width, int image_height, bool right) const
  {
    sensor_msgs::msg::CameraInfo info;
    info.header = header;
    info.width = image_width;
    info.height = image_height;
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    if (rectification_ready_ && rectification_size_.width == image_width &&
      rectification_size_.height == image_height)
    {
      const cv::Mat & r = right ? rectified_r2_ : rectified_r1_;
      const cv::Mat & p = right ? rectified_p2_ : rectified_p1_;
      info.k = {p.at<double>(0, 0), 0.0, p.at<double>(0, 2),
        0.0, p.at<double>(1, 1), p.at<double>(1, 2),
        0.0, 0.0, 1.0};
      info.r = {r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2),
        r.at<double>(1, 0), r.at<double>(1, 1), r.at<double>(1, 2),
        r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2)};
      info.p = {p.at<double>(0, 0), p.at<double>(0, 1), p.at<double>(0, 2), p.at<double>(0, 3),
        p.at<double>(1, 0), p.at<double>(1, 1), p.at<double>(1, 2), p.at<double>(1, 3),
        p.at<double>(2, 0), p.at<double>(2, 1), p.at<double>(2, 2), p.at<double>(2, 3)};
      return info;
    }

    const double fx = FocalXFor(image_width);
    const double fy = FocalYFor(image_width);
    const double cx = CenterXFor(image_width);
    const double cy = CenterYFor(image_height);

    info.k = {fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};
    info.p = {fx, 0.0, cx, right ? -fx * baseline_m_ : 0.0,
      0.0, fy, cy, 0.0,
      0.0, 0.0, 1.0, 0.0};
    return info;
  }

  geometry_msgs::msg::TransformStamped MakeStaticTransform(
    const std::string & parent, const std::string & child,
    double x, double y, double z,
    double roll, double pitch, double yaw) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;

    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    quaternion.normalize();
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();
    return transform;
  }

  void BuildStereoCalibration(
    int image_width, int image_height,
    cv::Mat *k1, cv::Mat *d1, cv::Mat *k2, cv::Mat *d2,
    cv::Mat *rotation, cv::Mat *translation) const
  {
    if (calibration_preset_ == "sdk_default") {
      constexpr double kDefaultWidth = 640.0;
      constexpr double kDefaultHeight = 400.0;
      constexpr double kDefaultBaseline = 0.12002489764113250;
      const double scale_x = static_cast<double>(image_width) / kDefaultWidth;
      const double scale_y = static_cast<double>(image_height) / kDefaultHeight;
      *k1 = (cv::Mat_<double>(3, 3) <<
        362.20059643202876 * scale_x, 0.0, 406.58699068023441 * scale_x,
        0.0, 363.50065250745848 * scale_y, 234.35161110061483 * scale_y,
        0.0, 0.0, 1.0);
      *k2 = k1->clone();
      *d1 = (cv::Mat_<double>(1, 5) <<
        -0.25034765682756088, 0.050579399202897619,
        -0.00070536671976066066, -0.0085255451307033846, 0.0);
      *d2 = d1->clone();
      *rotation = (cv::Mat_<double>(3, 3) <<
        0.99867908939669447, -0.0063445566137485428, 0.050988459509619687,
        0.0059890316389333252, 0.99995670037792639, 0.0071224201868366971,
        -0.051031440326695092, -0.0068076406092671274, 0.99867384471984544);
      const double baseline_scale = baseline_m_ / kDefaultBaseline;
      *translation = (cv::Mat_<double>(3, 1) <<
        -baseline_m_, -0.0011782637409050747 * baseline_scale,
        -0.0052058205159996538 * baseline_scale);
      return;
    }

    const double fx = FocalXFor(image_width);
    const double fy = FocalYFor(image_width);
    const double cx = CenterXFor(image_width);
    const double cy = CenterYFor(image_height);
    *k1 = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    *k2 = k1->clone();
    *d1 = cv::Mat::zeros(1, 5, CV_64F);
    *d2 = cv::Mat::zeros(1, 5, CV_64F);
    *rotation = cv::Mat::eye(3, 3, CV_64F);
    *translation = (cv::Mat_<double>(3, 1) << -baseline_m_, 0.0, 0.0);
  }

  bool EnsureStereoRectification(int image_width, int image_height)
  {
    if (!rectify_stereo_) {
      return false;
    }

    const cv::Size size(image_width, image_height);
    if (rectification_ready_ && rectification_size_ == size) {
      return true;
    }

    cv::Mat k1, d1, k2, d2, rotation, translation;
    BuildStereoCalibration(image_width, image_height, &k1, &d1, &k2, &d2,
                           &rotation, &translation);

    cv::Rect left_roi;
    cv::Rect right_roi;
    try {
      cv::stereoRectify(k1, d1, k2, d2, size, rotation, translation,
                        rectified_r1_, rectified_r2_, rectified_p1_, rectified_p2_,
                        reprojection_q_, cv::CALIB_ZERO_DISPARITY, 0.0, size,
                        &left_roi, &right_roi);
      cv::initUndistortRectifyMap(k1, d1, rectified_r1_, rectified_p1_, size,
                                  CV_32FC1, left_map_x_, left_map_y_);
      cv::initUndistortRectifyMap(k2, d2, rectified_r2_, rectified_p2_, size,
                                  CV_32FC1, right_map_x_, right_map_y_);
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Stereo rectification setup failed: %s", e.what());
      rectification_ready_ = false;
      return false;
    }

    rectification_size_ = size;
    rectified_fx_ = rectified_p1_.at<double>(0, 0);
    rectified_fy_ = rectified_p1_.at<double>(1, 1);
    rectified_cx_ = rectified_p1_.at<double>(0, 2);
    rectified_cy_ = rectified_p1_.at<double>(1, 2);
    rectified_baseline_m_ = std::abs(rectified_p2_.at<double>(0, 3) / rectified_fx_);
    rectification_ready_ = true;

    RCLCPP_INFO(get_logger(),
                "Stereo rectification ready (%s): fx %.2f fy %.2f cx %.2f cy %.2f baseline %.4f m",
                calibration_preset_.c_str(), rectified_fx_, rectified_fy_,
                rectified_cx_, rectified_cy_, rectified_baseline_m_);
    return true;
  }

  void AddStaticTransform(
    std::vector<geometry_msgs::msg::TransformStamped> *transforms,
    const std::string & parent, const std::string & child,
    double x, double y, double z,
    double roll = 0.0, double pitch = 0.0, double yaw = 0.0) const
  {
    if (parent.empty() || child.empty() || parent == child) {
      return;
    }
    transforms->push_back(MakeStaticTransform(parent, child, x, y, z, roll, pitch, yaw));
  }

  void PublishStaticTfTree()
  {
    if (!publish_tf_) {
      RCLCPP_INFO(get_logger(), "Static TF publishing disabled");
      return;
    }

    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    const double left_x = base_at_stereo_center_ ? -0.5 * baseline_m_ : 0.0;
    const double right_x = base_at_stereo_center_ ? 0.5 * baseline_m_ : baseline_m_;
    AddStaticTransform(&transforms, base_frame_, frame_id_left_, left_x, 0.0, 0.0);
    AddStaticTransform(&transforms, base_frame_, frame_id_right_, right_x, 0.0, 0.0);
    AddStaticTransform(&transforms, frame_id_left_, frame_id_points_, 0.0, 0.0, 0.0);
    AddStaticTransform(&transforms, base_frame_, frame_id_imu_, imu_x_m_, imu_y_m_, imu_z_m_);

    if (!transforms.empty()) {
      static_tf_broadcaster_->sendTransform(transforms);
      RCLCPP_INFO(get_logger(),
                  "Published static TF tree: %s -> {%s, %s, %s}",
                  base_frame_.c_str(), frame_id_left_.c_str(),
                  frame_id_right_.c_str(), frame_id_imu_.c_str());
    }
  }

  bool OpenCamera()
  {
    if (backend_ == "sdk") {
      return OpenSdkCamera();
    }
    return OpenUvcCamera();
  }

  bool OpenSdkCamera()
  {
    // API::Create with empty argv (we drive via ROS params)
    static char arg0[] = "mynteye";
    static char *argv[] = {arg0, nullptr};
    int argc = 1;
    api_ = API::Create(argc, argv);
    if (!api_) {return false;}

    bool ok = false;
    auto request = api_->SelectStreamRequest(&ok);
    if (!ok) {return false;}
    api_->ConfigStreamRequest(request);

    // Stream callbacks. Must NOT block.
    api_->SetStreamCallback(
        Stream::LEFT,
      [this](const api::StreamData & data) {OnImage(data, /*left=*/true);});
    api_->SetStreamCallback(
        Stream::RIGHT,
      [this](const api::StreamData & data) {OnImage(data, /*left=*/false);});

    api_->SetMotionCallback(
      [this](const api::MotionData & data) {OnImu(data);});

    api_->Start(Source::ALL);
    RCLCPP_INFO(get_logger(), "MYNT EYE camera started");
    return true;
  }

  std::vector<std::string> FindMyntVideoDevices() const
  {
    std::vector<std::string> devices;
    if (!video_device_.empty()) {
      devices.push_back(video_device_);
      return devices;
    }

    for (int i = 0; i < 64; ++i) {
      const std::string name_path = "/sys/class/video4linux/video" + std::to_string(i) + "/name";
      std::ifstream name_file(name_path);
      std::string name;
      std::getline(name_file, name);
      if (name.find("MYNT") == std::string::npos &&
        name.find("Mynteye") == std::string::npos)
      {
        continue;
      }

      const std::string index_path = "/sys/class/video4linux/video" + std::to_string(i) + "/index";
      std::ifstream index_file(index_path);
      int index = 0;
      if (index_file >> index && index != 0) {
        continue;
      }

      devices.push_back("/dev/video" + std::to_string(i));
    }
    return devices;
  }

  bool TryOpenUvcDevice(const std::string & device)
  {
    if (!std::filesystem::exists(device)) {
      RCLCPP_ERROR(get_logger(), "Configured V4L2 device does not exist: %s", device.c_str());
      return false;
    }

    cv::VideoCapture capture(device, cv::CAP_V4L2);
    if (!capture.isOpened()) {
      RCLCPP_WARN(get_logger(), "Could not open %s", device.c_str());
      return false;
    }

    capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    capture.set(cv::CAP_PROP_CONVERT_RGB, 0);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    capture.set(cv::CAP_PROP_FPS, fps_);
    capture.set(cv::CAP_PROP_BUFFERSIZE, 1);

    cv::Mat frame;
    for (int attempt = 0; attempt < 20; ++attempt) {
      if (capture.read(frame) && !frame.empty()) {
        RCLCPP_INFO(get_logger(),
                    "Opened MYNT EYE UVC stream on %s (%dx%d, channels=%d, layout=%s, %d fps requested)",
                    device.c_str(), frame.cols, frame.rows, frame.channels(),
                    uvc_stereo_layout_.c_str(), fps_);
        uvc_capture_ = std::move(capture);
        uvc_device_ = device;
        consecutive_uvc_failures_ = 0;
        last_uvc_frame_ = frame;
        return true;
      }
      std::this_thread::sleep_for(50ms);
    }

    RCLCPP_WARN(get_logger(), "Opened %s but no frames were received", device.c_str());
    return false;
  }

  bool OpenUvcCamera()
  {
    const auto wait_sec = std::max(0, device_wait_sec_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(wait_sec);
    bool saw_device = false;

    RCLCPP_INFO(get_logger(), "Waiting up to %d s for MYNT EYE V4L2 nodes", wait_sec);
    while (rclcpp::ok()) {
      auto devices = FindMyntVideoDevices();
      saw_device = saw_device || !devices.empty();

      if (!video_device_.empty() && !std::filesystem::exists(video_device_)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Configured V4L2 device does not exist yet: %s",
                             video_device_.c_str());
      } else {
        for (const auto & device : devices) {
          if (TryOpenUvcDevice(device)) {
            const auto period = std::chrono::milliseconds(std::max(1, 1000 / std::max(1, fps_)));
            uvc_timer_ = create_wall_timer(period, [this]() {PollUvcFrame();});
            RCLCPP_INFO(get_logger(), "MYNT EYE UVC backend started");
            return true;
          }
        }
      }

      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      rclcpp::sleep_for(250ms);
    }

    if (!saw_device) {
      RCLCPP_ERROR(get_logger(),
                   "No MYNT EYE V4L2 node found. Confirm that lsusb shows 1e4e:0120 "
                   "and that /dev/video* nodes exist before launching ROS.");
    } else {
      RCLCPP_ERROR(get_logger(), "MYNT EYE V4L2 nodes appeared, but none produced frames");
    }
    return false;
  }

  bool ReopenUvcCamera()
  {
    if (uvc_capture_.isOpened()) {
      uvc_capture_.release();
    }

    for (const auto & device : FindMyntVideoDevices()) {
      if (TryOpenUvcDevice(device)) {
        RCLCPP_INFO(get_logger(), "Recovered MYNT EYE UVC stream on %s", device.c_str());
        return true;
      }
    }
    return false;
  }

  void PollUvcFrame()
  {
    cv::Mat frame;
    if (!uvc_capture_.read(frame) || frame.empty()) {
      ++consecutive_uvc_failures_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No frame received from %s", uvc_device_.c_str());
      if (consecutive_uvc_failures_ >= 5) {
        RCLCPP_WARN(get_logger(), "UVC stream stalled; trying to reopen MYNT EYE video node");
        if (!ReopenUvcCamera()) {
          RCLCPP_WARN(get_logger(), "Could not recover MYNT EYE UVC stream yet");
        }
      }
      return;
    }
    consecutive_uvc_failures_ = 0;
    PublishUvcFrame(frame);
  }

  void PublishUvcFrame(const cv::Mat & frame)
  {
    const auto stamp = now();
    if (!split_stereo_ || frame.cols < 2) {
      std_msgs::msg::Header header;
      header.stamp = stamp;
      header.frame_id = frame_id_left_;
      auto msg = cv_bridge::CvImage(header, EncodingFor(frame), frame).toImageMsg();
      pub_left_->publish(*msg);
      return;
    }

    cv::Mat left_frame;
    cv::Mat right_frame;
    if (!ExtractStereoFrames(frame, &left_frame, &right_frame)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Could not extract MYNT EYE stereo images from UVC frame %dx%d channels=%d",
                           frame.cols, frame.rows, frame.channels());
      return;
    }

    if (swap_stereo_) {
      std::swap(left_frame, right_frame);
    }

    if (rectify_stereo_) {
      EnsureStereoRectification(left_frame.cols, left_frame.rows);
    }

    std_msgs::msg::Header left_header;
    left_header.stamp = stamp;
    left_header.frame_id = frame_id_left_;
    auto left_msg = cv_bridge::CvImage(
      left_header, EncodingFor(left_frame), left_frame).toImageMsg();
    pub_left_->publish(*left_msg);
    pub_left_info_->publish(BuildCameraInfo(left_header, left_frame.cols, left_frame.rows, false));

    std_msgs::msg::Header right_header;
    right_header.stamp = stamp;
    right_header.frame_id = frame_id_right_;
    auto right_msg = cv_bridge::CvImage(
      right_header, EncodingFor(right_frame), right_frame).toImageMsg();
    pub_right_->publish(*right_msg);
    pub_right_info_->publish(BuildCameraInfo(right_header, right_frame.cols, right_frame.rows,
        true));

    if (enable_depth_) {
      PublishStereoDepth(left_frame, right_frame, stamp);
    }
  }

  bool ExtractStereoFrames(const cv::Mat & frame, cv::Mat *left, cv::Mat *right) const
  {
    if (uvc_stereo_layout_ == "auto") {
      if (frame.type() == CV_8UC2 && frame.cols >= 1000) {
        return ExtractSideBySideStereo(frame, left, right);
      }
      if (frame.type() == CV_8UC2) {
        return ExtractInterleavedYuyvStereo(frame, left, right);
      }
      return ExtractSideBySideStereo(frame, left, right);
    }

    if (uvc_stereo_layout_ == "interleaved_yuyv") {
      return ExtractInterleavedYuyvStereo(frame, left, right);
    }

    if (uvc_stereo_layout_ == "side_by_side" ||
      uvc_stereo_layout_ == "side_by_side_yuyv")
    {
      return ExtractSideBySideStereo(frame, left, right);
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Unknown uvc_stereo_layout '%s'", uvc_stereo_layout_.c_str());
    return false;
  }

  bool ExtractInterleavedYuyvStereo(const cv::Mat & frame, cv::Mat *left, cv::Mat *right) const
  {
    if (frame.depth() != CV_8U) {
      return false;
    }

    if (frame.channels() == 2) {
      left->create(frame.rows, frame.cols, CV_8UC1);
      right->create(frame.rows, frame.cols, CV_8UC1);
      for (int y = 0; y < frame.rows; ++y) {
        const auto *src = frame.ptr<std::uint8_t>(y);
        auto *left_row = left->ptr<std::uint8_t>(y);
        auto *right_row = right->ptr<std::uint8_t>(y);
        for (int x = 0; x < frame.cols; ++x) {
          left_row[x] = src[2 * x];
          right_row[x] = src[2 * x + 1];
        }
      }
      return true;
    }

    if (frame.channels() == 1 && frame.cols >= 2 && frame.cols % 2 == 0) {
      const int stereo_width = frame.cols / 2;
      left->create(frame.rows, stereo_width, CV_8UC1);
      right->create(frame.rows, stereo_width, CV_8UC1);
      for (int y = 0; y < frame.rows; ++y) {
        const auto *src = frame.ptr<std::uint8_t>(y);
        auto *left_row = left->ptr<std::uint8_t>(y);
        auto *right_row = right->ptr<std::uint8_t>(y);
        for (int x = 0; x < stereo_width; ++x) {
          left_row[x] = src[2 * x];
          right_row[x] = src[2 * x + 1];
        }
      }
      return true;
    }

    return false;
  }

  bool ExtractSideBySideStereo(const cv::Mat & frame, cv::Mat *left, cv::Mat *right) const
  {
    if (frame.cols < 2) {
      return false;
    }
    const int half_width = frame.cols / 2;
    const cv::Rect left_roi(0, 0, half_width, frame.rows);
    const cv::Rect right_roi(half_width, 0, frame.cols - half_width, frame.rows);
    if (frame.type() == CV_8UC2) {
      cv::extractChannel(frame(left_roi), *left, 0);
      cv::extractChannel(frame(right_roi), *right, 0);
    } else {
      *left = frame(left_roi).clone();
      *right = frame(right_roi).clone();
    }
    return true;
  }

  std::string EncodingFor(const cv::Mat & frame) const
  {
    if (frame.channels() == 1) {
      return "mono8";
    }
    if (frame.channels() == 3) {
      return "bgr8";
    }
    if (frame.channels() == 4) {
      return "bgra8";
    }
    return "passthrough";
  }

  cv::Mat ToMono8(const cv::Mat & frame) const
  {
    cv::Mat gray;
    if (frame.channels() == 1) {
      gray = frame;
    } else if (frame.channels() == 3) {
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
      cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else {
      frame.convertTo(gray, CV_8U);
    }

    if (gray.depth() == CV_8U) {
      return gray;
    }

    cv::Mat gray8;
    gray.convertTo(gray8, CV_8U);
    return gray8;
  }

  cv::Mat PreprocessStereoImage(const cv::Mat & gray) const
  {
    if (!stereo_preprocess_) {
      return gray;
    }

    cv::Mat processed;
    if (clahe_clip_limit_ > 0.0) {
      const int tile_size = std::max(1, clahe_tile_grid_size_);
      auto clahe = cv::createCLAHE(clahe_clip_limit_, cv::Size(tile_size, tile_size));
      clahe->apply(gray, processed);
    } else {
      cv::equalizeHist(gray, processed);
    }
    return processed;
  }

  void ApplyDisparityMedian(cv::Mat *disparity_fixed) const
  {
    if (disparity_median_kernel_ < 3 || disparity_fixed->empty()) {
      return;
    }
    const int kernel = MakeOddAtLeast(disparity_median_kernel_, 3);
    cv::medianBlur(*disparity_fixed, *disparity_fixed, kernel);
  }

  void CleanValidMask(cv::Mat *valid_mask) const
  {
    if (valid_mask->empty()) {
      return;
    }

    if (depth_cleanup_kernel_ >= 3) {
      const int kernel_size = MakeOddAtLeast(depth_cleanup_kernel_, 3);
      const cv::Mat kernel = cv::getStructuringElement(
          cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
      cv::morphologyEx(*valid_mask, *valid_mask, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(*valid_mask, *valid_mask, cv::MORPH_CLOSE, kernel);
    }

    if (depth_min_component_area_ <= 1) {
      return;
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int label_count = cv::connectedComponentsWithStats(
        *valid_mask, labels, stats, centroids, 8, CV_32S);
    cv::Mat filtered = cv::Mat::zeros(valid_mask->size(), CV_8U);
    for (int label = 1; label < label_count; ++label) {
      const int area = stats.at<int>(label, cv::CC_STAT_AREA);
      if (area >= depth_min_component_area_) {
        filtered.setTo(255, labels == label);
      }
    }
    *valid_mask = filtered;
  }

  void ApplyMaskToFloatMat(cv::Mat *mat, const cv::Mat & valid_mask, float invalid_value) const
  {
    mat->setTo(invalid_value, valid_mask == 0);
  }

  std::string DepthOutputEncoding() const
  {
    std::string encoding = depth_image_encoding_;
    std::transform(encoding.begin(), encoding.end(), encoding.begin(),
      [](unsigned char c) {return std::tolower(c);});
    if (encoding == "16uc1" || encoding == "mono16") {
      return "16UC1";
    }
    return "32FC1";
  }

  cv::Mat ConvertDepthForPublish(const cv::Mat & depth_m, const std::string & encoding) const
  {
    if (encoding != "16UC1") {
      return depth_m;
    }

    cv::Mat depth_mm = cv::Mat::zeros(depth_m.size(), CV_16UC1);
    for (int y = 0; y < depth_m.rows; ++y) {
      const float *src = depth_m.ptr<float>(y);
      auto *dst = depth_mm.ptr<std::uint16_t>(y);
      for (int x = 0; x < depth_m.cols; ++x) {
        if (!std::isfinite(src[x]) || src[x] <= 0.0f) {
          continue;
        }
        const auto depth_mm_value = static_cast<int>(std::lround(src[x] * 1000.0f));
        if (depth_mm_value > 0 && depth_mm_value <= std::numeric_limits<std::uint16_t>::max()) {
          dst[x] = static_cast<std::uint16_t>(depth_mm_value);
        }
      }
    }
    return depth_mm;
  }

  void PublishPointCloud(
    const std_msgs::msg::Header & header,
    const cv::Mat & points_xyz,
    const cv::Mat & disparity,
    const cv::Mat & left_gray,
    const cv::Mat & right_gray)
  {
    const int stride = std::max(1, pointcloud_stride_);
    const int cloud_width = (points_xyz.cols + stride - 1) / stride;
    const int cloud_height = (points_xyz.rows + stride - 1) / stride;
    const float nan = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(static_cast<std::size_t>(cloud_width * cloud_height));
    cloud.width = cloud_width;
    cloud.height = cloud_height;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

    std::size_t valid_points = 0;
    for (int y = 0; y < points_xyz.rows; y += stride) {
      for (int x = 0; x < points_xyz.cols; x += stride) {
        const cv::Vec3f point = points_xyz.at<cv::Vec3f>(y, x);
        const float disp = disparity.at<float>(y, x);
        const bool valid = std::isfinite(point[0]) && std::isfinite(point[1]) &&
          std::isfinite(point[2]) && std::isfinite(disp) &&
          point[2] >= min_depth_m_ && point[2] <= max_depth_m_;

        if (valid) {
          if (pointcloud_ros_coordinates_) {
            *iter_x = point[2];
            *iter_y = -point[0];
            *iter_z = -point[1];
          } else {
            *iter_x = point[0];
            *iter_y = point[1];
            *iter_z = point[2];
          }
          const int right_x = static_cast<int>(std::lround(static_cast<double>(x) - disp));
          const float left_intensity = static_cast<float>(left_gray.at<std::uint8_t>(y, x));
          const float right_intensity =
            (right_x >= 0 && right_x < right_gray.cols) ?
            static_cast<float>(right_gray.at<std::uint8_t>(y, right_x)) :
            left_intensity;
          *iter_intensity = (0.5f * (left_intensity + right_intensity)) / 255.0f;
          ++valid_points;
        } else {
          *iter_x = nan;
          *iter_y = nan;
          *iter_z = nan;
          *iter_intensity = nan;
        }

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
      }
    }

    if (valid_points == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Stereo depth produced no valid point cloud samples");
    }
    pub_points_->publish(cloud);
  }

  void PublishStereoDepth(
    const cv::Mat & left_frame, const cv::Mat & right_frame, const rclcpp::Time & stamp)
  {
    if (!stereo_matcher_ || !right_stereo_matcher_) {
      return;
    }
    if (baseline_m_ <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Depth disabled until baseline_m is greater than zero");
      return;
    }

    const int common_width = std::min(left_frame.cols, right_frame.cols);
    const int common_height = std::min(left_frame.rows, right_frame.rows);
    if (common_width <= 0 || common_height <= 0) {
      return;
    }

    const cv::Rect common_roi(0, 0, common_width, common_height);
    const cv::Mat left_gray_raw = ToMono8(left_frame(common_roi));
    const cv::Mat right_gray_raw = ToMono8(right_frame(common_roi));

    cv::Mat left_rect = left_gray_raw;
    cv::Mat right_rect = right_gray_raw;
    const bool rectified = EnsureStereoRectification(common_width, common_height);
    if (rectified) {
      cv::remap(left_gray_raw, left_rect, left_map_x_, left_map_y_, cv::INTER_LINEAR);
      cv::remap(right_gray_raw, right_rect, right_map_x_, right_map_y_, cv::INTER_LINEAR);
    }

    std_msgs::msg::Header left_header;
    left_header.stamp = stamp;
    left_header.frame_id = frame_id_left_;
    std_msgs::msg::Header right_header;
    right_header.stamp = stamp;
    right_header.frame_id = frame_id_right_;

    if (publish_rectified_images_ && rectified) {
      pub_left_rect_->publish(
          *cv_bridge::CvImage(left_header, "mono8", left_rect).toImageMsg());
      pub_right_rect_->publish(
          *cv_bridge::CvImage(right_header, "mono8", right_rect).toImageMsg());
    }

    const cv::Mat left_match = PreprocessStereoImage(left_rect);
    const cv::Mat right_match = PreprocessStereoImage(right_rect);

    cv::Mat disparity_fixed;
    cv::Mat right_disparity_fixed;
    try {
      stereo_matcher_->compute(left_match, right_match, disparity_fixed);
      right_stereo_matcher_->compute(right_match, left_match, right_disparity_fixed);
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Stereo matcher failed: %s", e.what());
      return;
    }

    ApplyDisparityMedian(&disparity_fixed);
    ApplyDisparityMedian(&right_disparity_fixed);

    cv::Mat disparity;
    cv::Mat right_disparity_raw;
    disparity_fixed.convertTo(disparity, CV_32F, 1.0 / 16.0);
    right_disparity_fixed.convertTo(right_disparity_raw, CV_32F, 1.0 / 16.0);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    cv::Mat left_depth(disparity.size(), CV_32FC1, cv::Scalar(nan));
    cv::Mat right_depth(disparity.size(), CV_32FC1, cv::Scalar(nan));
    cv::Mat left_disparity(disparity.size(), CV_32FC1, cv::Scalar(nan));
    cv::Mat right_disparity(disparity.size(), CV_32FC1, cv::Scalar(nan));
    cv::Mat reproject_disparity(disparity.size(), CV_32FC1, cv::Scalar(nan));
    cv::Mat valid_mask(disparity.size(), CV_8UC1, cv::Scalar(0));

    const double fx = rectified ? rectified_fx_ : FocalXFor(common_width);
    const double fy = rectified ? rectified_fy_ : FocalYFor(common_width);
    const double cx = rectified ? rectified_cx_ : CenterXFor(common_width);
    const double cy = rectified ? rectified_cy_ : CenterYFor(common_height);
    const double baseline = rectified ? rectified_baseline_m_ : baseline_m_;

    for (int y = 0; y < disparity.rows; ++y) {
      const float *disparity_row = disparity.ptr<float>(y);
      float *left_depth_row = left_depth.ptr<float>(y);
      float *left_disparity_row = left_disparity.ptr<float>(y);
      float *reproject_disparity_row = reproject_disparity.ptr<float>(y);
      auto *valid_row = valid_mask.ptr<std::uint8_t>(y);
      for (int x = 0; x < disparity.cols; ++x) {
        const float disp = disparity_row[x];
        if (!std::isfinite(disp) || disp <= 0.5f) {
          continue;
        }

        const int right_x = static_cast<int>(std::lround(static_cast<double>(x) - disp));
        if (right_x < 0 || right_x >= right_disparity_raw.cols) {
          continue;
        }

        if (enable_lr_check_) {
          const float right_disp = right_disparity_raw.at<float>(y, right_x);
          if (!std::isfinite(right_disp) ||
            std::abs(static_cast<double>(disp + right_disp)) > lr_check_tolerance_px_)
          {
            continue;
          }
        }

        const float depth_m = static_cast<float>((fx * baseline) / disp);
        if (!std::isfinite(depth_m) || depth_m < min_depth_m_ || depth_m > max_depth_m_) {
          continue;
        }

        left_depth_row[x] = depth_m;
        left_disparity_row[x] = disp;
        reproject_disparity_row[x] = disp;
        valid_row[x] = 255;
      }
    }

    CleanValidMask(&valid_mask);
    ApplyMaskToFloatMat(&left_depth, valid_mask, nan);
    ApplyMaskToFloatMat(&left_disparity, valid_mask, nan);
    ApplyMaskToFloatMat(&reproject_disparity, valid_mask, nan);

    for (int y = 0; y < right_disparity_raw.rows; ++y) {
      const float *right_disparity_row = right_disparity_raw.ptr<float>(y);
      float *right_depth_row = right_depth.ptr<float>(y);
      float *right_disparity_out_row = right_disparity.ptr<float>(y);
      for (int x = 0; x < right_disparity_raw.cols; ++x) {
        const float disp = -right_disparity_row[x];
        if (!std::isfinite(disp) || disp <= 0.5f) {
          continue;
        }
        const float depth_m = static_cast<float>((fx * baseline) / disp);
        if (std::isfinite(depth_m) && depth_m >= min_depth_m_ && depth_m <= max_depth_m_) {
          right_depth_row[x] = depth_m;
          right_disparity_out_row[x] = disp;
        }
      }
    }

    cv::Mat points_xyz(reproject_disparity.size(), CV_32FC3, cv::Scalar(nan, nan, nan));
    if (rectified && !reprojection_q_.empty()) {
      cv::reprojectImageTo3D(reproject_disparity, points_xyz, reprojection_q_, false);
    } else {
      for (int y = 0; y < reproject_disparity.rows; ++y) {
        const float *disparity_row = reproject_disparity.ptr<float>(y);
        cv::Vec3f *point_row = points_xyz.ptr<cv::Vec3f>(y);
        for (int x = 0; x < reproject_disparity.cols; ++x) {
          const float disp = disparity_row[x];
          if (!std::isfinite(disp) || disp <= 0.5f) {
            continue;
          }
          const float depth_m = static_cast<float>((fx * baseline) / disp);
          point_row[x] = cv::Vec3f(
              static_cast<float>((static_cast<double>(x) - cx) * depth_m / fx),
              static_cast<float>((static_cast<double>(y) - cy) * depth_m / fy),
              depth_m);
        }
      }
    }

    pub_left_disparity_->publish(
        *cv_bridge::CvImage(left_header, "32FC1", left_disparity).toImageMsg());
    pub_right_disparity_->publish(
        *cv_bridge::CvImage(right_header, "32FC1", right_disparity).toImageMsg());
    const std::string depth_encoding = DepthOutputEncoding();
    pub_left_depth_->publish(
      *cv_bridge::CvImage(left_header, depth_encoding,
        ConvertDepthForPublish(left_depth, depth_encoding)).toImageMsg());
    pub_right_depth_->publish(
      *cv_bridge::CvImage(right_header, depth_encoding,
        ConvertDepthForPublish(right_depth, depth_encoding)).toImageMsg());

    if (publish_debug_images_) {
      cv::Mat disparity_vis;
      cv::Mat finite_disparity = left_disparity.clone();
      cv::patchNaNs(finite_disparity, 0.0);
      finite_disparity.convertTo(disparity_vis, CV_8U, 255.0 / std::max(1, num_disparities_));
      pub_debug_disparity_->publish(
          *cv_bridge::CvImage(left_header, "mono8", disparity_vis).toImageMsg());
    }

    if (publish_pointcloud_) {
      auto points_header = left_header;
      if (pointcloud_ros_coordinates_) {
        points_header.frame_id = frame_id_points_;
      }
      PublishPointCloud(points_header, points_xyz, left_disparity, left_rect, right_rect);
    }
  }

  rclcpp::Time TimestampToRosTime(std::uint64_t ts_us)
  {
    // Device timestamps are in microseconds from device start; we re-stamp
    // with the wall clock to keep it simple and TF-friendly. Users that need
    // hardware-synced stamps can swap this for a (epoch + ts) computation.
    (void)ts_us;
    return now();
  }

  void OnImage(const api::StreamData & data, bool left)
  {
    if (!data.frame.data) {return;}
    std_msgs::msg::Header header;
    header.stamp = TimestampToRosTime(data.img ? data.img->timestamp : 0);
    header.frame_id = left ? frame_id_left_ : frame_id_right_;

    // D1000 IR is mono8 (one channel).
    std::string encoding = (data.frame.channels() == 1) ? "mono8" : "bgr8";

    auto msg = cv_bridge::CvImage(header, encoding, data.frame).toImageMsg();
    if (left) {
      pub_left_->publish(*msg);
    } else {
      pub_right_->publish(*msg);
    }
  }

  void OnImu(const api::MotionData & data)
  {
    if (!data.imu) {return;}
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = TimestampToRosTime(data.imu->timestamp);
    msg.header.frame_id = frame_id_imu_;

    // SDK reports accel in g and gyro in deg/s.
    constexpr double kG = 9.80665;
    constexpr double kDegToRad = M_PI / 180.0;

    msg.linear_acceleration.x = data.imu->accel[0] * kG;
    msg.linear_acceleration.y = data.imu->accel[1] * kG;
    msg.linear_acceleration.z = data.imu->accel[2] * kG;

    msg.angular_velocity.x = data.imu->gyro[0] * kDegToRad;
    msg.angular_velocity.y = data.imu->gyro[1] * kDegToRad;
    msg.angular_velocity.z = data.imu->gyro[2] * kDegToRad;

    // No orientation provided by raw IMU.
    msg.orientation_covariance[0] = -1.0;

    pub_imu_->publish(msg);
  }

  std::shared_ptr<API> api_;
  cv::VideoCapture uvc_capture_;
  rclcpp::TimerBase::SharedPtr uvc_timer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  cv::Mat last_uvc_frame_;
  std::string backend_;
  std::string video_device_;
  std::string uvc_device_;
  cv::Ptr<cv::StereoMatcher> stereo_matcher_;
  cv::Ptr<cv::StereoMatcher> right_stereo_matcher_;
  cv::Size rectification_size_;
  bool rectification_ready_ = false;
  cv::Mat left_map_x_;
  cv::Mat left_map_y_;
  cv::Mat right_map_x_;
  cv::Mat right_map_y_;
  cv::Mat rectified_r1_;
  cv::Mat rectified_r2_;
  cv::Mat rectified_p1_;
  cv::Mat rectified_p2_;
  cv::Mat reprojection_q_;
  double rectified_fx_ = 0.0;
  double rectified_fy_ = 0.0;
  double rectified_cx_ = 0.0;
  double rectified_cy_ = 0.0;
  double rectified_baseline_m_ = 0.0;
  int width_;
  int height_;
  int fps_;
  int device_wait_sec_;
  std::string uvc_stereo_layout_;
  bool swap_stereo_;
  bool enable_depth_;
  bool publish_pointcloud_;
  bool publish_debug_images_;
  bool publish_rectified_images_;
  bool rectify_stereo_;
  std::string calibration_preset_;
  bool stereo_preprocess_;
  double clahe_clip_limit_;
  int clahe_tile_grid_size_;
  double baseline_m_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double horizontal_fov_deg_;
  double min_depth_m_;
  double max_depth_m_;
  std::string stereo_matcher_type_;
  int min_disparity_;
  int num_disparities_;
  int block_size_;
  int uniqueness_ratio_;
  int speckle_window_size_;
  int speckle_range_;
  int disp12_max_diff_;
  int pre_filter_cap_;
  int bm_pre_filter_size_;
  int bm_texture_threshold_;
  int disparity_median_kernel_;
  int depth_cleanup_kernel_;
  int depth_min_component_area_;
  std::string depth_image_encoding_;
  bool enable_lr_check_;
  double lr_check_tolerance_px_;
  int pointcloud_stride_;
  int consecutive_uvc_failures_;
  bool publish_tf_;
  std::string base_frame_;
  bool base_at_stereo_center_;
  bool pointcloud_ros_coordinates_;
  double imu_x_m_;
  double imu_y_m_;
  double imu_z_m_;
  bool split_stereo_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_rect_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_rect_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_disparity_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_disparity_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_disparity_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_;
  std::string frame_id_left_;
  std::string frame_id_right_;
  std::string frame_id_points_;
  std::string frame_id_imu_;
};

}  // namespace mynteye_camera

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<mynteye_camera::MyntEyeNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mynteye_camera_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
