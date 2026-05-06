cmake_minimum_required(VERSION 3.8)
project(mynteye_camera)

# C++17 minimum (matches SDK and ROS 2 Jazzy)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# --- ROS 2 deps ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# --- OpenCV (system, same one ROS Jazzy / cv_bridge use) ---
find_package(OpenCV 4 REQUIRED)

# --- Boost filesystem (re-exported as link interface by mynteye target) ---
find_package(Boost REQUIRED COMPONENTS filesystem)

# --- MYNT EYE S SDK (installed locally at <repo>/install) ---
# The SDK exports a CMake config: mynteye-config.cmake under lib/cmake/mynteye
set(MYNTEYE_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../install" CACHE PATH
    "Prefix where MYNT-EYE-S-SDK is installed")
list(APPEND CMAKE_PREFIX_PATH "${MYNTEYE_SDK_ROOT}")
find_package(mynteye REQUIRED CONFIG)
message(STATUS "Found MYNT EYE S-SDK: ${mynteye_VERSION} (${MYNTEYE_SDK_ROOT})")

# --- MYNT EYE D SDK (D1000 hardware depth, installed locally at <repo>/d_install) ---
set(MYNTEYE_D_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../d_install" CACHE PATH
    "Prefix where MYNT-EYE-D-SDK is installed")
list(APPEND CMAKE_PREFIX_PATH "${MYNTEYE_D_SDK_ROOT}")
find_package(mynteyed CONFIG QUIET)
if(mynteyed_FOUND)
  message(STATUS "Found MYNT EYE D-SDK: ${mynteyed_VERSION} (${MYNTEYE_D_SDK_ROOT})")
  set(D_SDK_AVAILABLE TRUE)
else()
  message(WARNING "MYNT EYE D-SDK not found at ${MYNTEYE_D_SDK_ROOT}. "
                  "Run scripts/build_d_sdk_ubuntu24.sh to build it. "
                  "D1000 hardware depth node will NOT be built.")
  set(D_SDK_AVAILABLE FALSE)
endif()

# --- S-series UVC node (S1030-IR and D1000 via OpenCV/V4L2 + software SGBM) ---
add_executable(mynteye_camera_node src/mynteye_camera_node.cpp)
target_include_directories(mynteye_camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${MYNTEYE_SDK_ROOT}/include
)
target_link_libraries(mynteye_camera_node
  ${OpenCV_LIBS}
  mynteye  # imported target from mynteye-targets.cmake
)
ament_target_dependencies(mynteye_camera_node
  rclcpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
)

# --- D1000 hardware depth node (requires MYNT-EYE-D-SDK) ---
if(D_SDK_AVAILABLE)
  # libeSPDI is in d_install/lib/3rdparty — set RPATH so the binary finds it
  set(ESPDI_LIB_DIR "${MYNTEYE_D_SDK_ROOT}/lib/3rdparty")

  add_executable(mynteye_d1000_node src/mynteye_d1000_node.cpp)
  target_include_directories(mynteye_d1000_node PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${MYNTEYE_D_SDK_ROOT}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../MYNT-EYE-D-SDK/_output/include
  )
  target_link_libraries(mynteye_d1000_node
    ${OpenCV_LIBS}
    mynteye_depth   # imported target from mynteyed-targets.cmake
  )
  # mynteyed-targets.cmake doesn't propagate WITH_OPENCV, so we add it manually
  target_compile_definitions(mynteye_d1000_node PRIVATE WITH_OPENCV)
  # Embed RPATH so the executable finds libeSPDI.so at runtime
  set_target_properties(mynteye_d1000_node PROPERTIES
    INSTALL_RPATH "${MYNTEYE_D_SDK_ROOT}/lib;${ESPDI_LIB_DIR}"
    BUILD_RPATH   "${MYNTEYE_D_SDK_ROOT}/lib;${ESPDI_LIB_DIR}"
  )
  ament_target_dependencies(mynteye_d1000_node
    rclcpp
    geometry_msgs
    sensor_msgs
    cv_bridge
    image_transport
    tf2
    tf2_ros
  )
  install(TARGETS mynteye_d1000_node
    DESTINATION lib/${PROJECT_NAME})
  message(STATUS "Building mynteye_d1000_node with hardware depth support")
endif()

install(TARGETS mynteye_camera_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(DIRECTORY config docs
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(FILES README.md LICENSE
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
