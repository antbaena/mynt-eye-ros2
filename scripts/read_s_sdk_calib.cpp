// Reads calibration from the MYNT EYE S-series camera EEPROM via the S-SDK
// and prints the values as ROS 2 launch arguments ready to paste.
//
// Build:
//   cd /home/mapir/mynt-eye-ros2
//   g++ -std=c++17 scripts/read_s_sdk_calib.cpp \
//       -I install/include \
//       -L install/lib -lmynteye \
//       -Wl,-rpath,install/lib \
//       -o /tmp/read_s_sdk_calib
//   /tmp/read_s_sdk_calib

#include <iostream>
#include <iomanip>
#include <memory>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  (void)argc; (void)argv;

  auto api = API::Create(0, nullptr);
  if (!api) {
    std::cerr << "ERROR: could not open camera. Check USB connection and udev rules.\n";
    return 1;
  }

  // Get intrinsics for LEFT and RIGHT streams
  auto in_left  = api->GetIntrinsics<IntrinsicsPinhole>(Stream::LEFT);
  auto in_right = api->GetIntrinsics<IntrinsicsPinhole>(Stream::RIGHT);
  auto ex       = api->GetExtrinsics(Stream::RIGHT, Stream::LEFT);

  std::cout << std::fixed << std::setprecision(8);
  std::cout << "\n=== MYNT EYE S-SDK calibration from EEPROM ===\n\n";
  std::cout << "LEFT  camera (" << in_left.width << "x" << in_left.height << "):\n"
            << "  fx=" << in_left.fx << "  fy=" << in_left.fy
            << "  cx=" << in_left.cx << "  cy=" << in_left.cy << "\n"
            << "  coeffs: k1=" << in_left.coeffs[0] << " k2=" << in_left.coeffs[1]
            << " p1=" << in_left.coeffs[2] << " p2=" << in_left.coeffs[3]
            << " k3=" << in_left.coeffs[4] << "\n\n";

  std::cout << "RIGHT camera (" << in_right.width << "x" << in_right.height << "):\n"
            << "  fx=" << in_right.fx << "  fy=" << in_right.fy
            << "  cx=" << in_right.cx << "  cy=" << in_right.cy << "\n"
            << "  coeffs: k1=" << in_right.coeffs[0] << " k2=" << in_right.coeffs[1]
            << " p1=" << in_right.coeffs[2] << " p2=" << in_right.coeffs[3]
            << " k3=" << in_right.coeffs[4] << "\n\n";

  // Baseline is the x-component of the translation vector (typically in meters)
  double baseline_m = std::abs(ex.translation[0]);
  std::cout << "Extrinsics (R->L):\n"
            << "  translation: [" << ex.translation[0] << ", "
            << ex.translation[1] << ", " << ex.translation[2] << "]\n"
            << "  baseline: " << baseline_m << " m\n\n";

  std::cout << "=== Ready to use as ROS 2 launch arguments ===\n\n"
            << "ros2 launch mynteye_camera mynteye.launch.py \\\n"
            << "  calibration_preset:=sdk_s1030_default \\\n"
            << "  width:=752 height:=480 \\\n"
            << "  fx:=" << in_left.fx << " \\\n"
            << "  fy:=" << in_left.fy << " \\\n"
            << "  cx:=" << in_left.cx << " \\\n"
            << "  cy:=" << in_left.cy << " \\\n"
            << "  baseline_m:=" << baseline_m << " \\\n"
            << "  device_wait_sec:=30\n\n";

  api->Stop(Source::ALL);
  return 0;
}
