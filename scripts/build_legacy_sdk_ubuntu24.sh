#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_DIR="${SDK_DIR:-${ROOT_DIR}/MYNT-EYE-S-SDK}"
SDK_REPO="${SDK_REPO:-https://github.com/slightech/MYNT-EYE-S-SDK.git}"
SDK_REF="${SDK_REF:-master}"
INSTALL_PREFIX="${MYNTEYE_SDK_PREFIX:-${ROOT_DIR}/install}"

if [[ ! -d "${SDK_DIR}/.git" ]]; then
  git clone "${SDK_REPO}" "${SDK_DIR}"
fi

git -C "${SDK_DIR}" fetch --tags --quiet || true
git -C "${SDK_DIR}" checkout "${SDK_REF}"
git -C "${SDK_DIR}" submodule update --init --recursive

python3 - "${SDK_DIR}" <<'PY'
from pathlib import Path
import sys

sdk = Path(sys.argv[1])


def replace_once(path, old, new):
    file_path = sdk / path
    text = file_path.read_text()
    if new in text:
        return
    if old not in text:
        raise SystemExit(f"Expected text not found in {path}")
    file_path.write_text(text.replace(old, new, 1))


replace_once(
    "CMakeLists.txt",
    "cmake_minimum_required(VERSION 3.0)\n\nproject(mynteye VERSION 2.5.0 LANGUAGES C CXX)",
    "cmake_minimum_required(VERSION 3.5)\n\nproject(mynteye VERSION 2.5.0 LANGUAGES C CXX)\n\n"
    "set(CMAKE_CXX_STANDARD 17)\n"
    "set(CMAKE_CXX_STANDARD_REQUIRED ON)\n"
    "set(CMAKE_CXX_EXTENSIONS OFF)",
)
replace_once(
    "CMakeLists.txt",
    "include(cmake/DetectCXX11.cmake)\n\nset(CMAKE_C_FLAGS",
    "include(cmake/DetectCXX11.cmake)\n\n"
    "string(REGEX REPLACE \"-std=[^ ]+\" \"\" CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS}\")\n"
    "set(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -std=c++17\")\n\n"
    "set(CMAKE_C_FLAGS",
)
replace_once(
    "CMakeLists.txt",
    "      src/mynteye/api/processor/rectify_processor.cc\n",
    "      # Disabled for OpenCV 4 / Ubuntu 24.04. D1000/S1030-IR use the PINHOLE OCV rectifier.\n"
    "      # src/mynteye/api/processor/rectify_processor.cc\n",
)
replace_once(
    "cmake/Option.cmake",
    "option(WITH_DEVICE_INFO_REQUIRED \"Build with device info required\" ON)",
    "option(WITH_DEVICE_INFO_REQUIRED \"Build with device info required\" OFF)",
)
replace_once(
    "cmake/Option.cmake",
    "  if(WITH_OPENCV4)\n    set(WITH_CAM_MODELS OFF)\n  endif()",
    "  if(WITH_OPENCV4)\n    # Keep camera models enabled for D1000/S1030-IR depth/points processors on OpenCV 4.\n    # set(WITH_CAM_MODELS OFF)\n  endif()",
)
replace_once(
    "src/mynteye/uvc/uvc.h",
    "#define MYNTEYE_PID 0x00F9\n",
    "#define MYNTEYE_PID 0x00F9\n\n"
    "#define MYNTEYE_VID_CUBETERNET 0x1E4E\n"
    "#define MYNTEYE_PID_D1000 0x0120  // D1000 (USB 1e4e:0120)\n"
    "#define MYNTEYE_PID_S1030 0x4722  // S1030-IR (USB 04b4:4722)\n",
)
replace_once(
    "src/mynteye/device/context.cc",
    "MYNTEYE_BEGIN_NAMESPACE\n\nContext::Context()",
    "MYNTEYE_BEGIN_NAMESPACE\n\nnamespace {\n\n"
    "bool is_mynteye_device(int vid, int pid) {\n"
    "  // MYNTEYE_VID (0x04B4, Cypress) already covers S1030-IR (04b4:4722).\n"
    "  return (vid == MYNTEYE_VID) ||\n"
    "      (vid == MYNTEYE_VID_CUBETERNET && pid == MYNTEYE_PID_D1000);\n"
    "}\n\n"
    "}  // namespace\n\nContext::Context()",
)
replace_once(
    "src/mynteye/device/context.cc",
    "if (vid == MYNTEYE_VID) {",
    "if (is_mynteye_device(vid, pid)) {",
)
replace_once(
    "src/mynteye/device/device.cc",
    "  if (name == \"MYNTEYE\") {\n    return std::make_shared<StandardDevice>(device);\n  } else if (strings::starts_with(name, \"MYNT-EYE-\")) {\n    std::string model_s = name.substr(9, 5);",
    "  if (name == \"MYNTEYE\") {\n    return std::make_shared<StandardDevice>(device);\n  }\n\n"
    "  std::string model_name = name;\n"
    "  auto model_pos = model_name.find(\"MYNT-EYE-\");\n"
    "  if (model_pos != std::string::npos) {\n"
    "    model_name = model_name.substr(model_pos);\n"
    "  }\n\n"
    "  if (model_name == \"MYNT-EYE-D1000\") {\n"
    "    return std::make_shared<StandardDevice>(device);\n"
    "  } else if (strings::starts_with(model_name, \"MYNT-EYE-\")) {\n"
    "    std::string model_s = model_name.substr(9, 5);",
)
replace_once(
    "src/mynteye/uvc/linux/uvc-v4l2.cc",
    "    if (!(std::ifstream(\"/sys/class/video4linux/\" + name + \"/name\") >>\n          this->name))\n      throw_error() << \"Failed to read name\";",
    "    std::ifstream name_file(\"/sys/class/video4linux/\" + name + \"/name\");\n"
    "    if (!std::getline(name_file, this->name) || this->name.empty())\n"
    "      throw_error() << \"Failed to read name\";",
)
replace_once(
    "src/mynteye/uvc/linux/uvc-v4l2.cc",
    "    try {\n      auto one_device = std::make_shared<device>(context, name);",
    "    int video_index = 0;\n"
    "    std::ifstream index_file(path + \"/index\");\n"
    "    if (index_file >> video_index) {\n"
    "      if (video_index != 0)\n"
    "        continue;\n"
    "    }\n\n"
    "    try {\n      auto one_device = std::make_shared<device>(context, name);",
)

(sdk / "src/mynteye/api/processor/rectify_processor.h").write_text(r'''// Patched for OpenCV 4 / Ubuntu 24.04 / ROS 2 Jazzy.
#ifndef MYNTEYE_API_PROCESSOR_RECTIFY_PROCESSOR_H_
#define MYNTEYE_API_PROCESSOR_RECTIFY_PROCESSOR_H_
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core/core.hpp>

#include "mynteye/api/processor.h"
#include "mynteye/device/device.h"

MYNTEYE_BEGIN_NAMESPACE

class Device;

class RectifyProcessor : public Processor {
 public:
  static constexpr const char NAME[] = "RectifyProcessor(stub)";

  RectifyProcessor(
      std::shared_ptr<IntrinsicsBase> /*intr_left*/,
      std::shared_ptr<IntrinsicsBase> /*intr_right*/,
      std::shared_ptr<Extrinsics> /*extr*/,
      std::int32_t proc_period = 0)
      : Processor(proc_period) {}
  virtual ~RectifyProcessor() {}

  std::string Name() override { return std::string("RectifyProcessor(stub)"); }

  void ReloadImageParams(
      std::shared_ptr<IntrinsicsBase> /*intr_left*/,
      std::shared_ptr<IntrinsicsBase> /*intr_right*/,
      std::shared_ptr<Extrinsics> /*extr*/) {}
  void ReloadImageParams() {}

  cv::Mat R1, P1, R2, P2, Q;
  cv::Mat map11, map12, map21, map22;

  inline std::shared_ptr<struct CameraROSMsgInfoPair> getCameraROSMsgInfoPair() {
    return calib_infos;
  }
  bool SetRectifyAlpha(float /*alpha*/) { return false; }

 protected:
  Object *OnCreateOutput() override { return nullptr; }
  bool OnProcess(
      Object *const /*in*/, Object *const /*out*/,
      std::shared_ptr<Processor> const /*parent*/) override { return false; }

 private:
  std::shared_ptr<struct CameraROSMsgInfoPair> calib_infos;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_PROCESSOR_RECTIFY_PROCESSOR_H_
''')
PY

cmake -S "${SDK_DIR}" -B "${SDK_DIR}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"
cmake --build "${SDK_DIR}/build" --target install -j"$(nproc)"

printf '\nMYNT EYE S SDK installed in: %s\n' "${INSTALL_PREFIX}"
