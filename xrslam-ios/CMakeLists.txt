cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-ios)

option(IOS_DEVELOPMENT_TEAM "Team ID for iOS app development" "XXXXXXXXXX")
option(APP_IDENTIFIER_PREFIX "Common prefix for all iOS app identifiers" "xrslam")

# Waypoint chantier W4: the upstream `visualizer` example app needs real
# codesigning (IOS_DEVELOPMENT_TEAM) and is a Swift app target, not a library
# -- neither is meaningful for a headless CI build of just the engine.
# Default ON so `build-ios.sh` (upstream's documented entry point) is
# unaffected; the CI workflow that builds this fork
# (rn/.github/workflows/ios-slam-engine.yml) passes it OFF.
option(XRSLAM_IOS_BUILD_VISUALIZER "Build the upstream example visualizer app" ON)

# Waypoint chantier W4: the WpSlamNative-equivalent Obj-C++ shim
# (native/slam/docs/SLAM-LIVE-CONTRACT.md Couche 1) that rn's SlamModuleIOS
# will eventually link against. Off by default (only meaningful once `xrslam`
# itself builds); the CI workflow turns it ON.
option(XRSLAM_IOS_BUILD_WPSLAM_SHIM "Build the wpslam_ios_shim Obj-C++ static library" OFF)

if(XRSLAM_IOS_BUILD_VISUALIZER AND TARGET xrslam AND TARGET xrslam-localization)
  add_subdirectory(visualizer)
endif()

if(XRSLAM_IOS_BUILD_WPSLAM_SHIM AND TARGET xrslam)
  add_subdirectory(wpslam)
endif()
