// WpSlamShim.h -- Waypoint chantier W4 (rn/native/slam/docs/SLAM-LIVE-CONTRACT.md,
// Couche 1, iOS mirror). Objective-C++ shim over libxrslam's C API
// (xrslam-interface/include/XRSLAM.h), method-for-method equivalent to the
// proven Android JNI shim (rn/native/slam/jni/wpslam_jni.cpp) -- same
// ingestion order, same one-active-session-at-a-time semantics, same
// two-attempt nativeCreate fallback. See WpSlamShim.mm's header comment for
// the full port rationale; this header only declares the ObjC surface
// `SlamModuleIOS.swift` (rn/modules/manual-camera/ios) is meant to call once
// this framework is linked into the RN app (see that file's TODO(slam-ios)).
//
// Deliberately NOT a 1:1 signature match of the Kotlin `external fun`
// contract (no opaque Long "handle" -- ObjC callers just hold a strong
// reference to one WpSlamEngine instance instead), because Swift/ObjC callers
// don't need a JNI-style handle indirection. The semantics are identical:
// XRSLAM's own C API is a process-global singleton, so only ONE WpSlamEngine
// instance may be "created" (active) at a time; a second -createWith...
// while one is already active returns NO without touching global state.

#import <Foundation/Foundation.h>

typedef NS_ENUM(NSInteger, WpSlamTrackingState) {
  WpSlamTrackingStateInitializing = 0,
  WpSlamTrackingStateTrackingSuccess = 1,
  WpSlamTrackingStateTrackingFail = 2,
};

NS_ASSUME_NONNULL_BEGIN

@interface WpSlamEngine : NSObject

/// Mirrors the JNI shim's nativeCreate: attempt 1 passes configYamlPath for
/// BOTH XRSLAMCreate's slam_config and device_config arguments; attempt 2
/// (fallback) writes the harness's byte-identical embedded slam_config to a
/// sibling file and retries with configYamlPath as device_config only.
/// Returns NO if a session is already active or if both attempts fail. Never
/// throws -- failures are logged via NSLog and reported through the BOOL
/// return only.
- (BOOL)createWithConfigYamlPath:(NSString *)configYamlPath;

/// 2026-08-26: diagnostic detail for the LAST -createWithConfigYamlPath: call
/// (which attempt(s) ran, whether each config file existed/its size/its
/// first bytes in hex, and the exact C++ exception type+message if either
/// attempt threw) -- nil only if -createWithConfigYamlPath: has never been
/// called. Exists so a caller can surface WHY creation failed through to a
/// user-visible/Sentry-visible string instead of only NSLog, which is
/// invisible on a TestFlight/production build with nobody attached in Xcode.
- (nullable NSString *)lastCreateDiagnostic;

/// No-op (logged) if this instance is not the active session. Safe to call
/// more than once.
- (void)destroy;

/// `gray` must point to `width*height` tightly-packed (stride == width)
/// 8-bit luma bytes -- the caller owns the buffer and it is only read for
/// the duration of this call (no async retention). No-op if this instance is
/// not the active session.
- (void)pushGray:(const uint8_t *)gray width:(int)width height:(int)height tNs:(int64_t)tNs;
- (void)pushAccelX:(float)x y:(float)y z:(float)z tNs:(int64_t)tNs;
- (void)pushGyroX:(float)x y:(float)y z:(float)z tNs:(int64_t)tNs;

/// WpSlamTrackingStateInitializing when this instance is not the active
/// session (neutral default, matches the contract's "0=INITIALIZING").
- (WpSlamTrackingState)state;

/// `out` must point to at least 7 floats (px,py,pz,qx,qy,qz,qw), the same
/// layout as the contract's `nativeGetPose` out array. Returns NO if no pose
/// is available yet (or this instance is not the active session).
- (BOOL)getPose:(float *)out;

/// Fills `out` (caller-allocated, capacity `maxPoints * 3` floats) with a
/// uniformly-decimated trajectory (oldest -> newest, same algorithm as the
/// JNI shim's nativeGetTrajectory), and returns the number of POINTS written
/// (0...maxPoints -- `out`'s consumed length is 3x the return value).
- (NSInteger)getTrajectory:(float *)out maxPoints:(NSInteger)maxPoints;

@end

NS_ASSUME_NONNULL_END
