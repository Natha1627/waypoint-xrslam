// WpSlamShim.mm -- Waypoint chantier W4 (rn/native/slam/docs/SLAM-LIVE-CONTRACT.md,
// Couche 1, iOS mirror).
//
// This is a DIRECT PORT of the proven, adversarially-reviewed Android JNI
// shim (rn/native/slam/jni/wpslam_jni.cpp, chantier A1, "build Modal vert du
// 1er essai" per the rn repo's cerebrum) -- same ingestion order (image push
// -> RunOneFrame once both IMU channels have flowed at least once; accel/gyro
// pushed inline), same one-active-session-at-a-time guard (XRSLAM.h's C API
// -- XRSLAMCreate/XRSLAMPushSensorData/XRSLAMRunOneFrame/XRSLAMGetResult/
// XRSLAMDestroy -- is a process-global singleton, NOT per-instance), same
// two-attempt nativeCreate fallback for the slam_config/device_config split
// (XRSLAMCreate wants two separate yaml paths; the Kotlin/Swift caller writes
// ONE merged yaml -- see wpslam_jni.cpp's header comment for the full
// deviation rationale, flagged there for the architect, unchanged here).
//
// Differences from the JNI version, and why:
//  - No JNIEnv / jobject plumbing -- this is a plain Objective-C++ class, not
//    a JNI extern "C" surface. `SlamModuleIOS.swift` will hold a strong
//    reference to one WpSlamEngine instance instead of a Long "handle".
//  - Errors log via NSLog instead of __android_log_print; same policy
//    otherwise (log + neutral return, never throw/abort, no exception can
//    escape a call made from Swift).
//
// "no exception can escape a call made from Swift" was stated here but NOT
// actually enforced until 2026-08-23: every XRSLAM* call below was
// unprotected. A production crash (Sentry, fatal, instant -- see rn's
// IOS-SLAM-WIRING.md) traced an uncaught yaml-cpp exception from
// XRSLAMCreate's device_config parsing straight through this file into
// Swift, which cannot catch a C++ exception crossing an Objective-C++
// boundary -- it crashes the whole process. Every XRSLAM* call site is now
// wrapped in try/catch (same idiom, same day, applied to wpslam_jni.cpp
// too), converting any throw into this file's existing log+neutral-return
// policy instead of letting it escape.
//
// VERIFIED: this file compiles, links, and has run successfully as part of
// a real signed build (App Store Connect, version 1.1 build 77, 2026-08-23,
// via rn's ios-testflight.yml) -- the crash above is itself proof it ran on
// a real device. No longer "unbuilt"; see rn/.github/workflows/
// ios-slam-engine.yml for the CI build/link path.
//
// 2026-08-26 -- the try/catch above did its job (no more crash on a fresh
// TestFlight build, run 32887684663 per rn's IOS-CRASH-DEMARRER-2026-08-25.md
// follow-up), but -createWithConfigYamlPath: now returns NO on-device
// ("suivi indisponible... createWithConfigYamlPath returned false"), meaning
// BOTH XRSLAMCreate attempts below still throw -- just caught instead of
// crashing. Two changes made investigating that, without a device/Mac
// available (same constraint as always for this chantier -- every claim
// below is a static-analysis finding, NOT a confirmed device fix):
//  1. `kFallbackSlamConfigYaml` (below) still carried the SAME malformed
//     `%YAML:1.0` directive line that caused the ORIGINAL crash (see the
//     2026-08-23 note above) -- byte-for-byte the text `SlamConfigYaml.swift`
//     already dropped from the PRIMARY generated config. Whether this line
//     alone is actually harmful to this yaml-cpp build is now genuinely
//     unclear (Android's Kotlin-generated config still emits it too, written
//     via a plain `std::ofstream`-equivalent with no NSString/Foundation
//     encoding step in between, same as this fallback constant -- and
//     Android does not reproduce this failure in the field, which argues the
//     directive line by itself is likely harmless and the ORIGINAL iOS crash
//     was more likely a Foundation string-encoding artifact specific to the
//     `String.write(...encoding:.utf8)` call already replaced by
//     `Data(utf8)` in `SlamModuleIOS.swift`). Removed anyway for zero-cost
//     parity with the primary generator -- there is no upside to leaving a
//     known-suspicious token in code that is otherwise byte-identical by
//     design, and it removes one variable from the next diagnosis regardless
//     of whether it was ever the actual cause.
//  2. `-lastCreateDiagnostic` (new): every attempt's outcome -- which
//     attempt, the exact path(s) used, whether each file existed on disk and
//     its byte size, the first 8 bytes of each file in hex (a BOM would show
//     up here immediately as `efbbbf...`), and the exact C++ exception
//     class+message if one was thrown -- is now accumulated into a single
//     diagnostic string instead of only ever reaching NSLog (invisible on a
//     TestFlight/production build with nobody attached in Xcode).
//     `SlamModuleIOS.swift` now reads this and folds it into the `reason`
//     string the JS layer already surfaces on-screen (the pill text the
//     founder read verbatim to report this bug) AND into the Sentry
//     breadcrumb `SlamCaptureScreen.tsx` already sends on this exact path --
//     so if this fix is incomplete, the NEXT tap of "Capturer" makes the
//     real cause visible with ZERO extra tooling, no new Sentry lookup, no
//     Xcode required.

#import "WpSlamShim.h"

#include "XRSLAM.h"

#include <cstdio>
#include <deque>
#include <exception>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

namespace {

// Byte-identical to xrslam-device-replay/src/main.cpp's kEmbeddedSlamConfigYaml
// / wpslam_jni.cpp's kFallbackSlamConfigYaml -- the "config yaml validated by
// the replay" the contract points at. Used ONLY as the fallback attempt.
//
// 2026-08-26: the leading `%YAML:1.0` line present in the Android/replay
// originals is deliberately DROPPED here (the one place this file already
// diverges byte-for-byte from its siblings) -- same reasoning as
// `SlamConfigYaml.swift`'s own removal of this line: it is not valid
// `%YAML major.minor` directive syntax, carries no numeric/semantic weight,
// and was the prime suspect (alongside a possible Foundation string-encoding
// artifact, since fixed) in the original 2026-08-23 crash. See this file's
// header comment for the full, honest uncertainty note -- this is a
// precautionary parity fix, not a confirmed-device fix.
const char *kFallbackSlamConfigYaml = R"YAML(output:
  q_bo: [ 0.0, 0.0, 0.0, 1.0 ] # x y z w
  p_bo: [ 0.0, 0.0, 0.0 ] # x y z [m]

sliding_window:
  size: 10
  subframe_size: 3
  force_keyframe_landmarks: 35

feature_tracker:
  min_keypoint_distance: 25.0 # [px]
  max_keypoint_detection: 200
  max_init_frames: 60
  max_frames: 20
  predict_keypoints: true
  clahe_clip_limit: 6.0
  clahe_width: 8
  clahe_height: 8

initializer:
  keyframe_num: 8
  keyframe_gap: 5
  min_matches: 50
  min_parallax: 10.0
  min_triangulation: 20
  min_landmarks: 30
  refine_imu: true

solver:
  iteration_limit: 30
  time_limit: 1.0e6 # [s]

rotation:
  misalignment_threshold: 0.02
  ransac_threshold: 10 # degree

parsac:
  parsac_flag: false
  dynamic_probability: 0.15
  threshold: 1.0
  norm_scale: 1.0
  keyframe_check_size: 1
)YAML";

// Same rationale as wpslam_jni.cpp: not contract-specified, sized for
// several minutes of session at the ~10-15 Hz effective VIO cadence.
constexpr size_t kTrajectoryRingCapacity = 8192;

struct Point3 {
    float x, y, z;
};

bool siblingPath(const std::string &configYamlPath, std::string &out) {
    size_t slash = configYamlPath.find_last_of('/');
    if (slash == std::string::npos) {
        return false;
    }
    out = configYamlPath.substr(0, slash + 1) + "_wpslam_fallback_slam_config.yaml";
    return true;
}

bool writeFallbackSlamConfig(const std::string &path) {
    std::ofstream f(path, std::ios::out | std::ios::trunc);
    if (!f.is_open()) {
        return false;
    }
    f << kFallbackSlamConfigYaml;
    f.close();
    return !f.fail();
}

// 2026-08-26 diagnostic (see this file's header comment): describes a config
// file EXACTLY as XRSLAMCreate/yaml-cpp would see it on this call, read
// independently of Swift's own write confirmation -- if Swift's
// `Data(yamlText.utf8).write(...)` reported success but this C++ side still
// cannot open/read the same path (sandbox path resolution mismatch, timing,
// anything), that shows up here as [MISSING/UNREADABLE] instead of silently
// falling through to a generic "returned false". The first-bytes hex dump
// exists specifically to catch a BOM or other leading-byte artifact ahead of
// yaml-cpp's scanner (the prime suspect in the original 2026-08-23 crash,
// see header comment) -- a UTF-8 BOM reads as `efbbbf...` here, unmistakably.
std::string describeConfigFile(const std::string &path) {
    std::ifstream f(path, std::ios::in | std::ios::binary);
    if (!f.is_open()) {
        return path + " [MISSING/UNREADABLE]";
    }
    f.seekg(0, std::ios::end);
    std::streamoff size = f.tellg();
    f.seekg(0, std::ios::beg);
    unsigned char head[8] = {0};
    f.read(reinterpret_cast<char *>(head), sizeof(head));
    std::streamsize got = f.gcount();
    std::ostringstream hex;
    hex << std::hex << std::setfill('0');
    for (std::streamsize i = 0; i < got; ++i) {
        hex << std::setw(2) << static_cast<int>(head[i]);
    }
    std::ostringstream out;
    out << path << " [size=" << size << "B first" << got << "B=" << hex.str() << "]";
    return out.str();
}

// XRSLAM has no per-instance handle -- it is a single global engine. Exactly
// one WpSlamEngine (the ObjC wrapper below) may be "active" at a time; this
// pointer identifies which one, so a stale/second instance's calls become
// safe no-ops instead of corrupting global XRSLAM state. Touched only from
// the single serial queue the Swift caller is contracted to use (mirrors the
// JNI shim's "the shim n'a pas besoin d'etre thread-safe" note) -- no locking.
void *g_activeToken = nullptr;

} // namespace

@interface WpSlamEngine () {
    void *_token; // this instance's identity token, compared against g_activeToken
    long long _epochNs;
    BOOL _hasAccel;
    BOOL _hasGyro;
    int _state;
    BOOL _hasPose;
    double _pose[7]; // px,py,pz,qx,qy,qz,qw
    std::deque<Point3> *_trajectory;
    std::string *_generatedSlamConfigPath;
    std::string *_lastCreateDiagnostic; // 2026-08-26, see -lastCreateDiagnostic
}
@end

@implementation WpSlamEngine

- (instancetype)init {
    self = [super init];
    if (self) {
        _token = nullptr;
        _epochNs = -1;
        _hasAccel = NO;
        _hasGyro = NO;
        _state = XRSLAM_STATE_INITIALIZING;
        _hasPose = NO;
        _trajectory = new std::deque<Point3>();
        _generatedSlamConfigPath = new std::string();
        _lastCreateDiagnostic = new std::string();
    }
    return self;
}

- (void)dealloc {
    if (_token != nullptr) {
        [self destroy];
    }
    delete _trajectory;
    delete _generatedSlamConfigPath;
    delete _lastCreateDiagnostic;
}

// 2026-08-26: see this file's header comment. Populated by every
// -createWithConfigYamlPath: call (success or failure) -- read by
// SlamModuleIOS.swift ONLY on failure today, but always fresh so a future
// caller could log it unconditionally without changing this method.
- (nullable NSString *)lastCreateDiagnostic {
    return _lastCreateDiagnostic->empty() ? nil : [NSString stringWithUTF8String:_lastCreateDiagnostic->c_str()];
}

- (BOOL)isActive {
    return _token != nullptr && _token == g_activeToken;
}

- (double)toSeconds:(int64_t)tNs {
    if (_epochNs < 0) {
        _epochNs = tNs;
    }
    return (double)(tNs - _epochNs) / 1e9;
}

- (void)pushTrajectoryPointX:(float)x y:(float)y z:(float)z {
    if (_trajectory->size() >= kTrajectoryRingCapacity) {
        _trajectory->pop_front();
    }
    _trajectory->push_back({x, y, z});
}

- (BOOL)createWithConfigYamlPath:(NSString *)configYamlPath {
    // 2026-08-26: accumulated regardless of outcome, exposed via
    // -lastCreateDiagnostic (see this file's header comment) -- every branch
    // below appends to it instead of (or in addition to) NSLog, so a
    // TestFlight/production build with nobody attached in Xcode still gets
    // the full picture through SlamModuleIOS.swift's `reason` string.
    std::ostringstream diag;
    _lastCreateDiagnostic->clear();

    if (g_activeToken != nullptr) {
        NSLog(@"[WpSlam] createWithConfigYamlPath: a session is already active -- call -destroy first");
        *_lastCreateDiagnostic = "session already active -- call destroy first";
        return NO;
    }
    std::string cfgPath = configYamlPath ? std::string(configYamlPath.UTF8String) : std::string();
    if (cfgPath.empty()) {
        NSLog(@"[WpSlam] createWithConfigYamlPath: configYamlPath is nil/empty");
        *_lastCreateDiagnostic = "configYamlPath is nil/empty";
        return NO;
    }

    void *configOut = nullptr;

    // Attempt 1: configYamlPath used for BOTH slam_config and device_config.
    //
    // try/catch (production crash, 2026-08-23 -- rn's IOS-SLAM-WIRING.md):
    // XRSLAMCreate can throw (e.g. yaml-cpp inside xrslam-extra's
    // device_config parsing, on a malformed config) instead of returning 0.
    // Swift cannot catch a C++ exception crossing an Objective-C++ boundary
    // -- if it escapes this method uncaught, it crashes the whole process,
    // which is exactly what happened. Converts a throw into the same
    // `created == 0` shape the rest of this function already handles, so no
    // downstream logic needs to change. Same fix applied to the JNI mirror
    // of this shim (rn's wpslam_jni.cpp) the same day.
    diag << "attempt1(combined) device_config=" << describeConfigFile(cfgPath);
    int created = 0;
    try {
        created = XRSLAMCreate(cfgPath.c_str(), cfgPath.c_str(), "", "Waypoint SLAM Live", &configOut);
    } catch (const std::exception &e) {
        NSLog(@"[WpSlam] createWithConfigYamlPath: XRSLAMCreate [combined attempt] threw: %s", e.what());
        diag << " threw(" << typeid(e).name() << "): " << e.what();
        created = 0;
    } catch (...) {
        NSLog(@"[WpSlam] createWithConfigYamlPath: XRSLAMCreate [combined attempt] threw a non-std exception");
        diag << " threw non-std-exception";
        created = 0;
    }
    if (created) {
        diag << " -> OK";
    } else if (diag.str().find("threw") == std::string::npos) {
        // XRSLAMCreate returned 0 WITHOUT throwing -- only possible today via
        // CheckLicense (hardcoded to always succeed upstream, so this branch
        // is not expected to fire in practice, but logged distinctly from a
        // thrown exception in case that ever changes).
        diag << " -> XRSLAMCreate returned 0 (no exception thrown)";
    }
    std::string generatedPath;

    if (!created) {
        NSLog(@"[WpSlam] createWithConfigYamlPath: XRSLAMCreate [combined attempt] failed -- "
               "retrying with the harness's fallback slam_config");

        if (!siblingPath(cfgPath, generatedPath)) {
            NSLog(@"[WpSlam] createWithConfigYamlPath: cannot derive a sibling path (no '/') -- fallback skipped");
            diag << " | attempt2 skipped: cannot derive sibling path from '" << cfgPath << "'";
        } else if (!writeFallbackSlamConfig(generatedPath)) {
            NSLog(@"[WpSlam] createWithConfigYamlPath: failed to write fallback slam_config to %s",
                  generatedPath.c_str());
            diag << " | attempt2 skipped: failed to write fallback slam_config to " << generatedPath;
            generatedPath.clear();
        } else {
            configOut = nullptr;
            diag << " | attempt2(fallback) slam_config=" << describeConfigFile(generatedPath)
                 << " device_config=" << describeConfigFile(cfgPath);
            try {
                created = XRSLAMCreate(generatedPath.c_str(), cfgPath.c_str(), "", "Waypoint SLAM Live", &configOut);
            } catch (const std::exception &e) {
                NSLog(@"[WpSlam] createWithConfigYamlPath: XRSLAMCreate [fallback attempt] threw: %s", e.what());
                diag << " threw(" << typeid(e).name() << "): " << e.what();
                created = 0;
            } catch (...) {
                NSLog(@"[WpSlam] createWithConfigYamlPath: XRSLAMCreate [fallback attempt] threw a non-std exception");
                diag << " threw non-std-exception";
                created = 0;
            }
            if (!created) {
                NSLog(@"[WpSlam] createWithConfigYamlPath: XRSLAMCreate [fallback attempt] also failed");
                std::remove(generatedPath.c_str());
                generatedPath.clear();
            } else {
                diag << " -> OK";
            }
        }
    }

    *_lastCreateDiagnostic = diag.str();

    if (!created) {
        NSLog(@"[WpSlam] createWithConfigYamlPath: both attempts failed");
        NSLog(@"[WpSlam] createWithConfigYamlPath: diagnostic: %s", _lastCreateDiagnostic->c_str());
        return NO;
    }

    // Any unique-per-instance address works as the identity token; __bridge
    // because this is a pointer-identity comparison only, never a retained
    // reference (ARC is enabled for this target).
    _token = (__bridge void *)self;
    *_generatedSlamConfigPath = generatedPath;
    g_activeToken = _token;

    NSLog(@"[WpSlam] createWithConfigYamlPath: session started%s",
          generatedPath.empty() ? "" : " [fallback slam_config used]");
    return YES;
}

- (void)destroy {
    if (![self isActive]) {
        NSLog(@"[WpSlam] destroy: not the active session -- no-op");
        return;
    }

    try {
        XRSLAMDestroy();
    } catch (const std::exception &e) {
        NSLog(@"[WpSlam] destroy: XRSLAMDestroy threw: %s -- continuing teardown anyway", e.what());
    } catch (...) {
        NSLog(@"[WpSlam] destroy: XRSLAMDestroy threw a non-std exception -- continuing teardown anyway");
    }

    if (!_generatedSlamConfigPath->empty()) {
        std::remove(_generatedSlamConfigPath->c_str());
        _generatedSlamConfigPath->clear();
    }

    g_activeToken = nullptr;
    _token = nullptr;
    NSLog(@"[WpSlam] destroy: session destroyed");
}

- (void)pushGray:(const uint8_t *)gray width:(int)width height:(int)height tNs:(int64_t)tNs {
    if (![self isActive]) {
        return;
    }
    if (gray == nullptr || width <= 0 || height <= 0) {
        NSLog(@"[WpSlam] pushGray: invalid args (width=%d height=%d)", width, height);
        return;
    }

    double tSeconds = [self toSeconds:tNs];

    // Same field layout as the harness's XRSLAMImage construction
    // (xrslam-device-replay/src/main.cpp) and the JNI shim: raw grayscale
    // pointer, tightly packed rows (stride = width -- the Swift caller hands
    // us an already-copied Y plane, not a live camera buffer with row
    // padding), single channel.
    XRSLAMImage image;
    image.data = const_cast<unsigned char *>(gray);
    image.timeStamp = tSeconds;
    image.stride = width;
    image.camera_id = 0;
    image.channel = 1;
    image.ext = nullptr;

    // try/catch around the whole block (production crash, 2026-08-23 -- see
    // createWithConfigYamlPath:'s comment above for the full story). Any of
    // PushSensorData/RunOneFrame/GetResult can throw from deep inside the
    // VIO pipeline (yaml-cpp, Ceres, OpenCV) at any point during a live
    // session, not just at creation -- Swift cannot catch a C++ exception
    // crossing this Objective-C++ boundary, so it must never escape here.
    try {
        XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA, &image);

        // Mirror the harness's own guard: only run the frame once IMU has flowed
        // at least once on both channels.
        if (_hasAccel && _hasGyro) {
            XRSLAMRunOneFrame();

            XRSLAMState state;
            XRSLAMGetResult(XRSLAM_RESULT_STATE, &state);
            _state = (int)state;

            if (state == XRSLAM_STATE_TRACKING_SUCCESS) {
                XRSLAMPose pose;
                // Every consumer of getPose: (world-space tie-point triangulation,
                // AR projection and trajectory display) requires T_world_camera.
                // BODY_POSE is the IMU/body pose and omits q_bc/p_bc; treating it
                // as a camera pose makes the reprojection error rotate with the
                // handset, so otherwise-static points appear to follow the view.
                // Keep this aligned with Android's wpslam_jni.cpp and XRSLAM's
                // own iOS visualizer, which both render CAMERA_POSE.
                XRSLAMGetResult(XRSLAM_RESULT_CAMERA_POSE, &pose);
                _pose[0] = pose.translation[0];
                _pose[1] = pose.translation[1];
                _pose[2] = pose.translation[2];
                _pose[3] = pose.quaternion[0];
                _pose[4] = pose.quaternion[1];
                _pose[5] = pose.quaternion[2];
                _pose[6] = pose.quaternion[3];
                _hasPose = YES;

                [self pushTrajectoryPointX:(float)pose.translation[0]
                                          y:(float)pose.translation[1]
                                          z:(float)pose.translation[2]];
            }
        }
    } catch (const std::exception &e) {
        NSLog(@"[WpSlam] pushGray: an XRSLAM call threw: %s", e.what());
    } catch (...) {
        NSLog(@"[WpSlam] pushGray: an XRSLAM call threw a non-std exception");
    }
}

- (void)pushAccelX:(float)x y:(float)y z:(float)z tNs:(int64_t)tNs {
    if (![self isActive]) {
        return;
    }
    double tSeconds = [self toSeconds:tNs];
    XRSLAMAcceleration acc = {{(double)x, (double)y, (double)z}, tSeconds};
    try {
        XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION, &acc);
        _hasAccel = YES;
    } catch (const std::exception &e) {
        NSLog(@"[WpSlam] pushAccelX: XRSLAMPushSensorData threw: %s", e.what());
    } catch (...) {
        NSLog(@"[WpSlam] pushAccelX: XRSLAMPushSensorData threw a non-std exception");
    }
}

- (void)pushGyroX:(float)x y:(float)y z:(float)z tNs:(int64_t)tNs {
    if (![self isActive]) {
        return;
    }
    double tSeconds = [self toSeconds:tNs];
    XRSLAMGyroscope gyr = {{(double)x, (double)y, (double)z}, tSeconds};
    try {
        XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE, &gyr);
        _hasGyro = YES;
    } catch (const std::exception &e) {
        NSLog(@"[WpSlam] pushGyroX: XRSLAMPushSensorData threw: %s", e.what());
    } catch (...) {
        NSLog(@"[WpSlam] pushGyroX: XRSLAMPushSensorData threw a non-std exception");
    }
}

- (WpSlamTrackingState)state {
    if (![self isActive]) {
        return WpSlamTrackingStateInitializing;
    }
    return (WpSlamTrackingState)_state;
}

- (BOOL)getPose:(float *)out {
    if (![self isActive] || !_hasPose || out == nullptr) {
        return NO;
    }
    for (int i = 0; i < 7; i++) {
        out[i] = (float)_pose[i];
    }
    return YES;
}

- (NSInteger)getTrajectory:(float *)out maxPoints:(NSInteger)maxPoints {
    if (![self isActive] || out == nullptr || maxPoints <= 0 || _trajectory->empty()) {
        return 0;
    }

    const size_t n = _trajectory->size();
    const size_t outN = (n <= (size_t)maxPoints) ? n : (size_t)maxPoints;

    if (outN == n) {
        size_t i = 0;
        for (const auto &p : *_trajectory) {
            out[i * 3 + 0] = p.x;
            out[i * 3 + 1] = p.y;
            out[i * 3 + 2] = p.z;
            i++;
        }
    } else if (outN == 1) {
        const Point3 &p = _trajectory->back();
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
    } else {
        // Uniform decimation across the whole buffer (oldest -> newest).
        for (size_t k = 0; k < outN; k++) {
            size_t idx = (size_t)((double)k * (double)(n - 1) / (double)(outN - 1) + 0.5);
            if (idx >= n) {
                idx = n - 1;
            }
            const Point3 &p = (*_trajectory)[idx];
            out[k * 3 + 0] = p.x;
            out[k * 3 + 1] = p.y;
            out[k * 3 + 2] = p.z;
        }
    }

    return (NSInteger)outN;
}

@end
