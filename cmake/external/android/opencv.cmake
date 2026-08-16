if(NOT TARGET depends::opencv)
  # Mirrors cmake/external/ios/opencv.cmake's approach (FetchContent a
  # precompiled, official OpenCV release archive rather than cross-compiling
  # OpenCV ourselves): the iOS module fetches opencv-<ver>-ios-framework.zip,
  # this one fetches the equivalent official precompiled Android SDK archive,
  # opencv-<ver>-android-sdk.zip, which ships prebuilt static libs for every
  # Android ABI plus the CMake config package needed by find_package(OpenCV).
  #
  # This is the missing link SuperBuildDepends.cmake's superbuild_extern()
  # already dispatches to: cmake/Modules/SuperBuildDepends.cmake sets
  # SUPERBUILD_EXTERN_PLATFORM to ANDROID whenever the CMake ANDROID variable
  # is set (i.e. the NDK toolchain file is in play) and looks for exactly
  # this file (cmake/external/android/<extern_name>.cmake) before falling
  # back to the plain find_package(OpenCV REQUIRED) in cmake/external/
  # opencv.cmake, which has no Android SDK equivalent to find on a cross
  # build (there is no system OpenCV under the NDK sysroot).
  set(_xrslam_opencv_android_version "4.10.0")
  FetchContent_Declare(
    depends-opencv
    URL https://github.com/opencv/opencv/releases/download/${_xrslam_opencv_android_version}/opencv-${_xrslam_opencv_android_version}-android-sdk.zip
  )
  FetchContent_GetProperties(depends-opencv)
  if(NOT depends-opencv_POPULATED)
    message(STATUS "Fetching precompiled OpenCV Android SDK ${_xrslam_opencv_android_version}")
    FetchContent_Populate(depends-opencv)
    message(STATUS "Fetching precompiled OpenCV Android SDK ${_xrslam_opencv_android_version} - done")
  endif()

  # The official opencv-<ver>-android-sdk.zip contains a single top-level
  # "OpenCV-android-sdk" directory. FetchContent's zip extraction (cmake -E
  # tar xf, no tar-style --strip-components) does not unwrap that, unlike a
  # hand-rolled `unzip` into a directory named after the archive would
  # suggest -- probe both the nested and flat layouts instead of assuming
  # one, so this keeps working if that ever changes upstream.
  set(_xrslam_opencv_candidate_roots
    "${depends-opencv_SOURCE_DIR}/OpenCV-android-sdk"
    "${depends-opencv_SOURCE_DIR}"
  )

  set(_xrslam_opencv_jni_dir "")
  foreach(_xrslam_opencv_root ${_xrslam_opencv_candidate_roots})
    # Prefer the ABI-specific config dir over the top-level jni/
    # OpenCVConfig.cmake (which dispatches on $ANDROID_ABI itself): a bare
    # `find_package(OpenCV)` pointed at the dispatcher has been observed
    # elsewhere in this project's NDK spikes to resolve a *different* ABI's
    # static libs at configure time and only fail at link
    # ("is incompatible with aarch64linux") -- see
    # native/slam/tools/spikes/supereight2_ndk_spike.py's OpenCVConfig.cmake
    # note. Pointing OpenCV_DIR directly at jni/abi-<ABI> removes that
    # ambiguity for the one ABI this build actually targets.
    if(EXISTS "${_xrslam_opencv_root}/sdk/native/jni/abi-${ANDROID_ABI}/OpenCVConfig.cmake")
      set(_xrslam_opencv_jni_dir "${_xrslam_opencv_root}/sdk/native/jni/abi-${ANDROID_ABI}")
      break()
    elseif(EXISTS "${_xrslam_opencv_root}/sdk/native/jni/OpenCVConfig.cmake")
      set(_xrslam_opencv_jni_dir "${_xrslam_opencv_root}/sdk/native/jni")
      break()
    endif()
  endforeach()

  if(NOT _xrslam_opencv_jni_dir)
    message(FATAL_ERROR
      "cmake/external/android/opencv.cmake: could not locate OpenCVConfig.cmake "
      "under ${depends-opencv_SOURCE_DIR} (checked both the nested "
      "OpenCV-android-sdk/ and flat layouts, for ANDROID_ABI=${ANDROID_ABI}) -- "
      "the opencv-${_xrslam_opencv_android_version}-android-sdk.zip layout may "
      "have changed upstream."
    )
  endif()

  # Wire OpenCV_DIR to sdk/native/jni (or its ABI-specific subdirectory) so
  # find_package(OpenCV) below resolves against it.
  set(OpenCV_DIR "${_xrslam_opencv_jni_dir}" CACHE PATH "OpenCV Android SDK config dir" FORCE)
  message(STATUS "Using precompiled OpenCV Android SDK: OpenCV_DIR=${OpenCV_DIR}")

  # The NDK toolchain file sets CMAKE_FIND_ROOT_PATH_MODE_PACKAGE=ONLY by
  # default, which restricts find_package() to CMAKE_FIND_ROOT_PATH (the NDK
  # sysroot) and would otherwise ignore OpenCV_DIR entirely. Explicit PATHS +
  # NO_DEFAULT_PATH bypasses that restriction instead of relaxing the global
  # find-root policy -- the same trick this project's supereight2 NDK spike
  # used to feed Eigen3_DIR/Boost_INCLUDE_DIR past the identical restriction.
  find_package(OpenCV REQUIRED CONFIG PATHS "${OpenCV_DIR}" NO_DEFAULT_PATH)

  add_library(depends::opencv INTERFACE IMPORTED GLOBAL)
  target_link_libraries(depends::opencv INTERFACE ${OpenCV_LIBS})
endif()
