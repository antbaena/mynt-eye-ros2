#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mynteye_depth" for configuration "Release"
set_property(TARGET mynteye_depth APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(mynteye_depth PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/mapir/mynt-eye-ros2/d_install/lib/libmynteye_depth.so.1.9.0"
  IMPORTED_SONAME_RELEASE "libmynteye_depth.so.1"
  )

list(APPEND _cmake_import_check_targets mynteye_depth )
list(APPEND _cmake_import_check_files_for_mynteye_depth "/home/mapir/mynt-eye-ros2/d_install/lib/libmynteye_depth.so.1.9.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
