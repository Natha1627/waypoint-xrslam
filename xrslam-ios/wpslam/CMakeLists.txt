cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

# Waypoint chantier W4 (rn/native/slam/docs/SLAM-LIVE-CONTRACT.md Couche 1,
# iOS mirror of the Android JNI shim / chantier A1). A small Objective-C++
# static library over the `xrslam` interface target built two directories up
# (xrslam-ios/CMakeLists.txt only add_subdirectory()s this when
# XRSLAM_IOS_BUILD_WPSLAM_SHIM is ON -- see that file).
enable_language(OBJCXX)

add_library(wpslam_ios_shim STATIC
  WpSlamShim.h
  WpSlamShim.mm
)

target_include_directories(wpslam_ios_shim
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# `xrslam` (xrslam-interface/CMakeLists.txt, XRSLAM_IOS branch) already
# publishes XRSLAM.h + links every static dependency this shim's .mm needs
# (xrslam-core, xrslam-extra-opencv-image, xrslam-extra-yaml-config,
# yaml-cpp -- merged via that target's post-build `libtool -static` step).
target_link_libraries(wpslam_ios_shim
  PUBLIC
    xrslam
)

set_target_properties(wpslam_ios_shim PROPERTIES
  OBJCXX_STANDARD 17
  CXX_STANDARD 17
)
