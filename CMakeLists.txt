cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-superbuild)

include(CMakeDependentOption)
include(SuperBuildDepends)
superbuild_option(cpp17)
superbuild_option(modern-cpp)

option(XRSLAM_DEBUG "..." ON)
option(XRSLAM_ENABLE_DEBUG_INSPECTION "..." ON)
option(XRSLAM_ENABLE_THREADING "..." OFF)
option(XRSLAM_PC_HEADLESS_ONLY "..." OFF)
option(XRSLAM_IOS "..." OFF)
option(XRSLAM_IOS_SYNCHRONOUS_CORE
       "Run the iOS feature tracker/frontend synchronously like the validated Android build"
       ON)
option(XRSLAM_TEST "..." OFF)
option(XRSLAM_DEVICE_REPLAY "Waypoint: build xrslam_device_replay headless CLI (xrslam-device-replay/)" OFF)

if(IOS)
  set(XRSLAM_IOS ON)
  # Waypoint production default: match the Android execution model.  The
  # upstream iOS-only threaded path can publish an IMU prediction while its
  # visual backend is still catching up; worse, its per-frame PnP fast path
  # used to construct a Solver without ever calling solve().  Keeping the
  # core synchronous makes XRSLAMPushSensorData/RunOneFrame causally complete
  # before the wrapper publishes a pose, exactly like the Android build that
  # has been validated on-device.  The old pipeline remains available for a
  # deliberate benchmark with -DXRSLAM_IOS_SYNCHRONOUS_CORE=OFF.
  if(XRSLAM_IOS_SYNCHRONOUS_CORE)
    set(XRSLAM_ENABLE_THREADING OFF)
  else()
    set(XRSLAM_ENABLE_THREADING ON)
  endif()
  #set(XRSLAM_ENABLE_DEBUG_INSPECTION OFF CACHE BOOL "..." FORCE)
endif()


if(NOT IOS)
  set(XRSLAM_PC OFF)
endif()

add_subdirectory(xrslam)
add_subdirectory(xrslam-extra)
add_subdirectory(xrslam-interface)
add_subdirectory(xrslam-localization)

if(XRSLAM_DEVICE_REPLAY)
  add_subdirectory(xrslam-device-replay)
endif()

if(XRSLAM_PC)
  add_subdirectory(xrslam-pc)
endif()
if(XRSLAM_IOS)
  add_subdirectory(xrslam-ios)
endif()
if(XRSLAM_TEST)
  add_subdirectory(xrslam-test)
endif()
