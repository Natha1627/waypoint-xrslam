cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
project(xrslam-device-replay)

# On-device headless replay CLI (Waypoint mission, native/slam PLAN.md Porte
# 1). Statically links xrslam-core + the xrslam-interface C-API sources
# (compiled directly into this executable, NOT via the shared `xrslam`
# target from ../xrslam-interface/CMakeLists.txt -- that target stays
# SHARED/untouched, this is a second, independent consumer of the same
# static xrslam-core / xrslam-extra-* / depends::opencv libraries) + the
# Android SDK's static OpenCV libs (depends::opencv, wired by
# cmake/external/android/opencv.cmake -- same path libxrslam.so already
# uses). Only runtime dependency left is libc++_shared.so (ANDROID_STL=
# c++_shared, project-wide) + bionic system libs.
add_executable(xrslam_device_replay
  src/main.cpp
  ${CMAKE_SOURCE_DIR}/xrslam-interface/src/XRSLAMInternal.cpp
  ${CMAKE_SOURCE_DIR}/xrslam-interface/src/XRSLAMManager.cpp
)

target_include_directories(xrslam_device_replay PRIVATE
  ${CMAKE_SOURCE_DIR}/xrslam-interface/include
  ${CMAKE_SOURCE_DIR}/xrslam-interface/src
)

target_link_libraries(xrslam_device_replay PRIVATE
  xrslam-extra-opencv-image
  xrslam-extra-yaml-config
  xrslam-core
)

set(XRSLAM_DEVICE_REPLAY_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
