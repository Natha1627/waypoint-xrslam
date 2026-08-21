// xrslam_device_replay -- on-device (Android arm64) headless CLI.
//
// Waypoint mission (native/slam/PLAN.md, Porte 1): measure XRSLAM/RD-VIO's
// REAL inter-frame latency on the founder's Redmi, not a desktop/x86 proxy.
// Replays a pre-extracted packet (frames/ + imu.jsonl + config.yaml) through
// XRSLAM's C API and prints latency/state/trajectory stats as JSON.
//
// Timing methodology matches bug-102 (rn/.wolf/buglog.json) and this
// project's other XRSLAM runners (tools/spikes/xrslam_build_spike.py,
// tools/spikes/xrslam_packet_replay.py): XRSLAMRunOneFrame() alone only
// enqueues the frame + a cheap IMU-predicted pose (~microseconds) -- the
// real feature-tracking cost is DEFERRED into the *next*
// XRSLAMPushSensorData(GYROSCOPE/ACCELERATION) call (Detail::track_imu()).
// We time every XRSLAM API call and attribute the WHOLE inter-camera-frame
// window (image push + RunOneFrame + every IMU push up to the next image
// push) to that frame -- exactly like the Linux spikes, so numbers are
// directly comparable to the 20.6/47.2 ms (mean/p95) x86 reference.
//
// Usage:
//   xrslam_device_replay --input <dir> [--out <file.json>] \
//                         [--max-frames N] [--slam-config <path>] \
//                         [--dump-jsonl <file.jsonl>]
//
// --dump-jsonl (Waypoint chantier R2, AR-overlay convention verification
// bench, native/slam/bench/ar_verify/): one JSON object PER PROCESSED FRAME
// (not the aggregated --out summary), written incrementally. Fields:
//   frame_index, frame_path (basename under frames/), t_ns (ABSOLUTE, same
//   epoch as imu.jsonl's "t" and the frame filename -- no epoch subtraction,
//   unlike the --out trajectory's relative "t"), t_s (relative to this run's
//   epoch, kept for continuity with --out), state (0/1/2), state_name,
//   pose_valid (state==TRACKING_SUCCESS), body_pose {p:[x,y,z],
//   q_xyzw:[x,y,z,w]}, camera_pose {p, q_xyzw} (same shape), n_landmarks,
//   landmarks_xyz (flat world-frame xyz, decimated to --dump-max-landmarks).
//
// POSE SEMANTICS (read from xrslam-interface/src/XRSLAMManager.cpp, NOT from
// XRSLAM.h's doxygen comment on XRSLAMPose -- that comment, "for a 3D point
// in world coordinate X_w, its 3D coordinate in the camera frame X_c = R *
// X_w + T", is the OPPOSITE of what the implementation actually returns and
// must not be trusted):
//   GetResultBodyPose():   body_pose.q = latest_pose.q * imu_to_body_rotation();
//                          body_pose.p = latest_pose.p + latest_pose.q * imu_to_body_translation();
//   GetResultCameraPose(): camera_pose.q = latest_pose.q * camera_to_body_rotation();  // config's q_bc
//                          camera_pose.p = latest_pose.p + latest_pose.q * camera_to_body_translation();
// where `latest_pose` is the sliding window's own optimized IMU/body pose.
// This is a forward (LOCAL-TO-WORLD) compose: p_world = R * p_local + t, i.e.
// XRSLAMPose.translation is WHERE the body/camera IS in the XRSLAM world
// frame, and XRSLAMPose.quaternion (Eigen coeffs order, x,y,z,w) rotates
// LOCAL-frame vectors INTO world frame -- exactly "T_world_body" /
// "T_world_camera" in the R2 mission's H1/H2 vocabulary, NOT "T_body_world".
// To project a world point into image space you therefore need the INVERSE:
// p_camera = R(q_world_camera)^T * (p_world - t_world_camera).
// camera_to_body_rotation()/translation() are read straight from this
// packet's device_config.yaml (cam0.extrinsic.q_bc/p_bc) -- i.e. GetResult-
// CameraPose already applies the REAL calibrated Tcb for this capture. The
// live product path (jni/wpslam_jni.cpp, native/slam/jni/) instead calls
// GetResultBodyPose only and is expected to apply its OWN "Tcb portrait_
// known" downstream (docs/SLAM-LIVE-CONTRACT.md) -- comparing body_pose (+
// externally-applied Tcb) against camera_pose (Tcb applied internally, known
// correct for this device) is exactly the H1 vs H2/H3/H4/H5 cross-check this
// dump feeds.
//
// LANDMARKS: XRSLAMLandmark only carries {x,y,z} (world frame) -- verified
// in xrslam/include/xrslam/inspection.h's `struct Landmark {vector<3> p;
// bool triangulated;}` (populated by initializer.cpp/sliding_window_
// tracker.cpp) and in xrslam-interface's GetResultLandmarks: even though the
// underlying `Track` class DOES carry a stable `Identifiable<Track>::id()`
// internally, that id is never copied into the Landmark/XRSLAMLandmark
// structs, and `map->get_track(i)` iterates dense STORAGE SLOTS (reused
// across evicted tracks), not a stable identity -- so there is no
// (unmodified-API) way to correlate "the same physical landmark" across two
// dumped frames by id. This dump therefore emits xyz only, per-frame; the R2
// analysis script re-derives persistence via a spatial hash instead (the
// point of the mission's "magasin d'ancres persistant").
//
// <dir> must contain:
//   frames/f_NNNNN_<t_ns>.jpg -- JPEGs; filename embeds the ALREADY-joined
//                                pose-instant timestamp in nanoseconds (the
//                                desktop prep script does the PTS<->t_ns
//                                join per PACKET.md -- this binary trusts
//                                the filename, no further correction here).
//   imu.jsonl                 -- plain (NOT gzipped) NDJSON: one header line
//                                (ignored -- has no "t" field) then one
//                                sample per line, accel
//                                {"t":ns,"ax":..,"ay":..,"az":..} or gyro
//                                {"t":ns,"gx":..,"gy":..,"gz":..}.
//   config.yaml                -- XRSLAM device_config.yaml (cam0
//                                intrinsics/distortion/extrinsic + imu
//                                noise). The slam_config.yaml (sliding-
//                                window/feature-tracker params) is EMBEDDED
//                                below -- identical across every run so far
//                                (native/slam/bench/p1_replay/summary.json)
//                                -- override with --slam-config if needed.
#include "XRSLAM.h"
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

static std::string json_number(double value) {
    if (!std::isfinite(value))
        return "null";
    std::ostringstream out;
    out << std::setprecision(12) << value;
    return out.str();
}

namespace {

// Sliding-window / feature-tracker / initializer params -- IDENTICAL to
// native/slam/tools/spikes/xrslam_packet_replay.py's SLAM_CONFIG_YAML,
// proven across every Porte-1 replay so far (bench/p1_replay/summary.json).
// Embedded (not shipped in the packet) because it never changes per-device;
// only device_config.yaml (calibration) does.
const char *kEmbeddedSlamConfigYaml = R"YAML(%YAML:1.0
output:
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

struct FrameItem {
    long long t_ns;
    std::string path;
    int index;
};

struct ImuItem {
    long long t_ns;
    char kind; // 'a' (accel) or 'g' (gyro)
    double x, y, z;
};

// Minimal hand-rolled field extraction for our own controlled, flat NDJSON
// lines ({"t":123,"ax":0.1,...}) -- avoids pulling in a JSON library for a
// fixed, self-produced schema. Int64 for "t" (nanosecond epoch values
// overflow double's 2^53 exact-integer range) -- everything else is a plain
// double.
bool extract_i64(const std::string &line, const char *key, long long &out) {
    std::string needle = std::string("\"") + key + "\":";
    size_t pos = line.find(needle);
    if (pos == std::string::npos)
        return false;
    pos += needle.size();
    while (pos < line.size() && line[pos] == ' ')
        pos++;
    char *endp = nullptr;
    long long v = std::strtoll(line.c_str() + pos, &endp, 10);
    if (endp == line.c_str() + pos)
        return false;
    out = v;
    return true;
}

bool extract_f64(const std::string &line, const char *key, double &out) {
    std::string needle = std::string("\"") + key + "\":";
    size_t pos = line.find(needle);
    if (pos == std::string::npos)
        return false;
    pos += needle.size();
    while (pos < line.size() && line[pos] == ' ')
        pos++;
    char *endp = nullptr;
    double v = std::strtod(line.c_str() + pos, &endp);
    if (endp == line.c_str() + pos)
        return false;
    out = v;
    return true;
}

// frames/f_NNNNN_<t_ns>.jpg -- historical naming (native/slam bench
// captures). Sorted by t_ns (not filename order) -- the mission's own
// instruction ("pousse IMU+frames dans l'ordre des timestamps").
std::vector<FrameItem> list_frames(const std::string &frames_dir) {
    std::vector<FrameItem> items;
    DIR *d = opendir(frames_dir.c_str());
    if (!d)
        return items;
    struct dirent *ent;
    while ((ent = readdir(d)) != nullptr) {
        std::string name = ent->d_name;
        if (name.size() < 5 || name.substr(0, 2) != "f_")
            continue;
        size_t dot = name.rfind('.');
        if (dot == std::string::npos)
            continue;
        std::string ext = name.substr(dot);
        for (auto &c : ext)
            c = (char)tolower((unsigned char)c);
        if (ext != ".jpg" && ext != ".jpeg")
            continue;
        std::string stem = name.substr(2, dot - 2); // strip "f_" prefix + ext
        size_t us = stem.find('_');
        if (us == std::string::npos)
            continue;
        std::string idx_str = stem.substr(0, us);
        std::string tns_str = stem.substr(us + 1);
        char *endp = nullptr;
        int idx = (int)std::strtol(idx_str.c_str(), &endp, 10);
        long long tns = std::strtoll(tns_str.c_str(), &endp, 10);
        if (tns <= 0)
            continue;
        items.push_back({tns, frames_dir + "/" + name, idx});
    }
    closedir(d);
    std::sort(items.begin(), items.end(),
               [](const FrameItem &a, const FrameItem &b) { return a.t_ns < b.t_ns; });
    return items;
}

std::vector<ImuItem> load_imu_jsonl(const std::string &path) {
    std::vector<ImuItem> items;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty())
            continue;
        long long t_ns;
        if (!extract_i64(line, "t", t_ns))
            continue; // header/trailer lines have no "t" field -- skip.
        double ax, ay, az, gx, gy, gz;
        if (extract_f64(line, "ax", ax) && extract_f64(line, "ay", ay) &&
            extract_f64(line, "az", az)) {
            items.push_back({t_ns, 'a', ax, ay, az});
        } else if (extract_f64(line, "gx", gx) && extract_f64(line, "gy", gy) &&
                   extract_f64(line, "gz", gz)) {
            items.push_back({t_ns, 'g', gx, gy, gz});
        }
    }
    std::sort(items.begin(), items.end(),
               [](const ImuItem &a, const ImuItem &b) { return a.t_ns < b.t_ns; });
    return items;
}

double percentile(const std::vector<double> &sorted_vals, double p) {
    if (sorted_vals.empty())
        return 0.0;
    size_t idx = (size_t)(p * (double)(sorted_vals.size() - 1));
    return sorted_vals[idx];
}

} // namespace

int main(int argc, char **argv) {
    std::string input_dir, out_path, slam_cfg_override, dump_jsonl_path;
    long max_frames = -1;
    long dump_max_landmarks = 2000;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                fprintf(stderr, "missing value for %s\n", a.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--input")
            input_dir = next();
        else if (a == "--out")
            out_path = next();
        else if (a == "--max-frames")
            max_frames = std::stol(next());
        else if (a == "--slam-config")
            slam_cfg_override = next();
        else if (a == "--dump-jsonl")
            dump_jsonl_path = next();
        else if (a == "--dump-max-landmarks")
            dump_max_landmarks = std::stol(next());
        else {
            fprintf(stderr, "unknown arg: %s\n", a.c_str());
            return 2;
        }
    }
    if (input_dir.empty()) {
        fprintf(stderr,
                "usage: %s --input <dir> [--out <file.json>] [--max-frames N] "
                "[--slam-config <path>] [--dump-jsonl <file.jsonl>] "
                "[--dump-max-landmarks N]\n",
                argv[0]);
        return 2;
    }
    while (!input_dir.empty() && input_dir.back() == '/')
        input_dir.pop_back();

    std::string device_cfg_path = input_dir + "/config.yaml";
    std::string imu_path = input_dir + "/imu.jsonl";
    std::string frames_dir = input_dir + "/frames";

    std::string slam_cfg_path = slam_cfg_override;
    std::string generated_slam_cfg_path;
    if (slam_cfg_path.empty()) {
        generated_slam_cfg_path = "./_xrslam_slam_config_generated.yaml";
        std::ofstream sc(generated_slam_cfg_path);
        sc << kEmbeddedSlamConfigYaml;
        sc.close();
        slam_cfg_path = generated_slam_cfg_path;
    }

    // Own K/D copy for pre-undistortion -- mirrors this project's other
    // XRSLAM runners (xrslam-pc/player's reference reader undistorts with
    // the SAME K/D that live in device_config.yaml before pushing to
    // XRSLAM; XRSLAMCreate below reads device_config.yaml a second time
    // internally, via yaml-cpp, for its own geometry pipeline).
    cv::FileStorage fs(device_cfg_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        fprintf(stderr, "cannot open device config %s\n", device_cfg_path.c_str());
        return 2;
    }
    cv::FileNode cam0 = fs["cam0"];
    std::vector<double> intr, dist;
    cam0["intrinsics"] >> intr;
    cam0["distortion"] >> dist;
    int distortion_flag = (int)cam0["camera_distortion_flag"];
    fs.release();
    if (intr.size() < 4) {
        fprintf(stderr, "config.yaml: cam0.intrinsics malformed (need 4 values)\n");
        return 2;
    }
    if (dist.size() < 4)
        dist = {0.0, 0.0, 0.0, 0.0};
    cv::Mat K =
        (cv::Mat_<float>(3, 3) << intr[0], 0, intr[2], 0, intr[1], intr[3], 0, 0, 1);
    cv::Mat D = (cv::Mat_<float>(4, 1) << dist[0], dist[1], dist[2], dist[3]);

    auto frames = list_frames(frames_dir);
    auto imu = load_imu_jsonl(imu_path);
    fprintf(stderr, "loaded %zu camera frames, %zu imu samples from %s\n", frames.size(),
            imu.size(), input_dir.c_str());
    if (frames.empty()) {
        fprintf(stderr, "no frames found under %s (expected f_NNNNN_<t_ns>.jpg)\n",
                frames_dir.c_str());
        return 2;
    }
    if (imu.empty()) {
        fprintf(stderr, "no imu samples found in %s\n", imu_path.c_str());
        return 2;
    }

    long long epoch_ns = std::min(frames.front().t_ns, imu.front().t_ns);

    void *yaml_config = nullptr;
    int created = XRSLAMCreate(slam_cfg_path.c_str(), device_cfg_path.c_str(), "",
                                "Waypoint XRSLAM Device Replay", &yaml_config);
    fprintf(stderr, "XRSLAMCreate ok=%d\n", created);
    if (!created) {
        if (!generated_slam_cfg_path.empty())
            std::remove(generated_slam_cfg_path.c_str());
        return 1;
    }

    size_t ci = 0, ii = 0;
    bool has_gyro = false, has_accel = false;
    long processed = 0;
    long tracking_success = 0;
    long state_counts[3] = {0, 0, 0}; // INITIALIZING, TRACKING_SUCCESS, TRACKING_FAIL

    double accum_ms = 0.0;
    long open_frame_idx = -1;
    long long open_frame_t_ns = 0;
    std::string open_frame_path;
    int pending_state = -1;
    double pending_pose[7] = {0, 0, 0, 0, 0, 0, 1};        // body: tx,ty,tz,qx,qy,qz,qw
    double pending_camera_pose[7] = {0, 0, 0, 0, 0, 0, 1}; // camera: same layout
    bool pending_camera_pose_valid = false;
    std::vector<double> pending_landmarks_xyz; // flat world-frame xyz
    std::vector<XRSLAMQualifiedLandmark> pending_qualified_landmarks;

    std::ofstream dump_jsonl_stream;
    if (!dump_jsonl_path.empty()) {
        dump_jsonl_stream.open(dump_jsonl_path, std::ios::out | std::ios::trunc);
        if (!dump_jsonl_stream.is_open()) {
            fprintf(stderr, "cannot open --dump-jsonl output %s\n", dump_jsonl_path.c_str());
            return 2;
        }
        dump_jsonl_stream << std::setprecision(9);
    }

    std::vector<double> frame_times_ms;
    struct TrajRow {
        long long t_ns;
        int state;
        double p[3];
        double q[4];
        double frame_ms;
    };
    std::vector<TrajRow> trajectory;

    auto timed = [](auto &&fn) {
        auto t0 = std::chrono::steady_clock::now();
        fn();
        auto t1 = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    };

    // Closes the PREVIOUS frame's accumulation window: it now includes every
    // IMU push that ran since that frame's image push, which is where its
    // deferred feature-tracking actually executed (see TIMING NOTE above).
    static const char *kStateNames[3] = {"INITIALIZING", "TRACKING_SUCCESS", "TRACKING_FAIL"};
    auto flush_pending = [&]() {
        if (open_frame_idx < 0)
            return;
        frame_times_ms.push_back(accum_ms);
        if (pending_state >= 0 && pending_state <= 2)
            state_counts[pending_state]++;
        trajectory.push_back({open_frame_t_ns, pending_state,
                               {pending_pose[0], pending_pose[1], pending_pose[2]},
                               {pending_pose[3], pending_pose[4], pending_pose[5], pending_pose[6]},
                               accum_ms});

        if (dump_jsonl_stream.is_open()) {
            const char *state_name =
                (pending_state >= 0 && pending_state <= 2) ? kStateNames[pending_state] : "UNKNOWN";
            std::ostringstream row;
            row << std::setprecision(9);
            row << "{\"frame_index\":" << open_frame_idx
                << ",\"frame_path\":\"" << open_frame_path << "\""
                << ",\"t_ns\":" << open_frame_t_ns
                << ",\"t_s\":" << (double)(open_frame_t_ns - epoch_ns) / 1e9
                << ",\"state\":" << pending_state
                << ",\"state_name\":\"" << state_name << "\""
                << ",\"pose_valid\":" << (pending_camera_pose_valid ? "true" : "false")
                << std::setprecision(9)
                << ",\"body_pose\":{\"p\":[" << pending_pose[0] << "," << pending_pose[1] << ","
                << pending_pose[2] << "],\"q_xyzw\":[" << pending_pose[3] << "," << pending_pose[4]
                << "," << pending_pose[5] << "," << pending_pose[6] << "]}"
                << ",\"camera_pose\":{\"p\":[" << pending_camera_pose[0] << ","
                << pending_camera_pose[1] << "," << pending_camera_pose[2] << "],\"q_xyzw\":["
                << pending_camera_pose[3] << "," << pending_camera_pose[4] << ","
                << pending_camera_pose[5] << "," << pending_camera_pose[6] << "]}"
                << ",\"n_landmarks\":" << (pending_landmarks_xyz.size() / 3)
                << ",\"landmarks_xyz\":[";
            for (size_t k = 0; k < pending_landmarks_xyz.size(); k++) {
                if (k)
                    row << ",";
                row << pending_landmarks_xyz[k];
            }
            row << "]"
                << ",\"n_qualified_landmarks\":" << pending_qualified_landmarks.size()
                << ",\"qualified_landmarks\":[";
            for (size_t k = 0; k < pending_qualified_landmarks.size(); ++k) {
                if (k)
                    row << ",";
                const XRSLAMQualifiedLandmark &q = pending_qualified_landmarks[k];
                row << "{\"track_id\":" << q.track_id
                    << ",\"xyz\":[" << json_number(q.x) << ","
                    << json_number(q.y) << "," << json_number(q.z) << "]"
                    << ",\"inverse_depth\":" << json_number(q.inverse_depth)
                    << ",\"triangulation_angle_rad\":"
                    << json_number(q.triangulation_angle_rad)
                    << ",\"mean_reprojection_error_px\":"
                    << json_number(q.mean_reprojection_error_px)
                    << ",\"max_reprojection_error_px\":"
                    << json_number(q.max_reprojection_error_px)
                    << ",\"observation_px\":[" << json_number(q.observation_u_px)
                    << "," << json_number(q.observation_v_px) << "]"
                    << ",\"first_t\":" << json_number(q.first_observation_timestamp)
                    << ",\"last_t\":" << json_number(q.last_observation_timestamp)
                    << ",\"observation_count\":" << q.observation_count
                    << ",\"life\":" << q.life
                    << ",\"valid\":" << (q.valid ? "true" : "false")
                    << ",\"triangulated\":" << (q.triangulated ? "true" : "false")
                    << ",\"outlier\":" << (q.outlier ? "true" : "false")
                    << ",\"static\":" << (q.static_track ? "true" : "false")
                    << "}";
            }
            row << "]"
                << ",\"frame_ms\":" << std::setprecision(4) << accum_ms << std::setprecision(9)
                << "}\n";
            dump_jsonl_stream << row.str();
        }
    };

    while (ci < frames.size() || ii < imu.size()) {
        bool take_imu;
        if (ii >= imu.size())
            take_imu = false;
        else if (ci >= frames.size())
            take_imu = true;
        else
            take_imu = imu[ii].t_ns <= frames[ci].t_ns;

        if (take_imu) {
            const ImuItem &it = imu[ii++];
            double t_s = (double)(it.t_ns - epoch_ns) / 1e9;
            if (it.kind == 'g') {
                XRSLAMGyroscope gyr = {{it.x, it.y, it.z}, t_s};
                accum_ms += timed([&] { XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE, &gyr); });
                has_gyro = true;
            } else {
                XRSLAMAcceleration acc = {{it.x, it.y, it.z}, t_s};
                accum_ms +=
                    timed([&] { XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION, &acc); });
                has_accel = true;
            }
        } else {
            if (max_frames >= 0 && processed >= max_frames)
                break;
            const FrameItem &it = frames[ci++];
            cv::Mat raw = cv::imread(it.path, cv::IMREAD_GRAYSCALE);
            if (raw.empty()) {
                fprintf(stderr, "warn: unreadable frame %s\n", it.path.c_str());
                continue;
            }
            cv::Mat img;
            if (distortion_flag)
                cv::undistort(raw, img, K, D);
            else
                img = raw;

            flush_pending();
            accum_ms = 0.0;
            open_frame_idx = processed;
            open_frame_t_ns = it.t_ns;
            {
                size_t slash = it.path.find_last_of('/');
                open_frame_path = (slash == std::string::npos) ? it.path : it.path.substr(slash + 1);
            }
            pending_state = -1;
            pending_pose[0] = pending_pose[1] = pending_pose[2] = 0;
            pending_pose[3] = pending_pose[4] = pending_pose[5] = 0;
            pending_pose[6] = 1;
            pending_camera_pose[0] = pending_camera_pose[1] = pending_camera_pose[2] = 0;
            pending_camera_pose[3] = pending_camera_pose[4] = pending_camera_pose[5] = 0;
            pending_camera_pose[6] = 1;
            pending_camera_pose_valid = false;
            pending_landmarks_xyz.clear();
            pending_qualified_landmarks.clear();

            double t_s = (double)(it.t_ns - epoch_ns) / 1e9;
            XRSLAMImage image;
            image.data = img.data;
            image.timeStamp = t_s;
            image.stride = (int)img.step[0];
            image.camera_id = 0;
            image.channel = img.channels();
            image.ext = nullptr;
            accum_ms += timed([&] { XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA, &image); });

            if (has_gyro && has_accel) {
                accum_ms += timed([&] { XRSLAMRunOneFrame(); });
                processed++;

                XRSLAMState state;
                XRSLAMGetResult(XRSLAM_RESULT_STATE, &state);
                pending_state = (int)state;
                if (state == XRSLAM_STATE_TRACKING_SUCCESS) {
                    tracking_success++;
                    XRSLAMPose pose;
                    XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE, &pose);
                    pending_pose[0] = pose.translation[0];
                    pending_pose[1] = pose.translation[1];
                    pending_pose[2] = pose.translation[2];
                    pending_pose[3] = pose.quaternion[0];
                    pending_pose[4] = pose.quaternion[1];
                    pending_pose[5] = pose.quaternion[2];
                    pending_pose[6] = pose.quaternion[3];

                    // Camera pose (Tcb ALREADY applied internally, from this
                    // packet's device_config.yaml cam0.extrinsic.q_bc/p_bc --
                    // see the pose-semantics note atop this file). Fetched
                    // alongside body pose so the R2 dump can compare H1
                    // (this) against H2..H5 (body pose + externally-applied
                    // Tcb hypotheses) on the exact same instant.
                    XRSLAMPose cam_pose;
                    XRSLAMGetResult(XRSLAM_RESULT_CAMERA_POSE, &cam_pose);
                    pending_camera_pose[0] = cam_pose.translation[0];
                    pending_camera_pose[1] = cam_pose.translation[1];
                    pending_camera_pose[2] = cam_pose.translation[2];
                    pending_camera_pose[3] = cam_pose.quaternion[0];
                    pending_camera_pose[4] = cam_pose.quaternion[1];
                    pending_camera_pose[5] = cam_pose.quaternion[2];
                    pending_camera_pose[6] = cam_pose.quaternion[3];
                    pending_camera_pose_valid = true;
                }

                // Landmarks: attempt on EVERY processed frame (not just
                // TRACKING_SUCCESS) so the dump itself reveals when the
                // debug-inspection slot is empty/stale vs. freshly populated
                // (see LANDMARKS note atop this file). GetResultLandmarks new[]s
                // the array and never frees it (xrslam-interface, confirmed by
                // jni/wpslam_jni.cpp's own deviation notes) -- owned+freed here.
                if (!dump_jsonl_path.empty()) {
                    XRSLAMLandmarks lm;
                    lm.landmarks = nullptr;
                    lm.num_landmarks = 0;
                    try {
                        XRSLAMGetResult(XRSLAM_RESULT_LANDMARKS, &lm);
                    } catch (...) {
                        lm.landmarks = nullptr;
                        lm.num_landmarks = 0;
                    }
                    if (lm.landmarks != nullptr && lm.num_landmarks > 0) {
                        long n = lm.num_landmarks;
                        long keep = (dump_max_landmarks > 0 && n > dump_max_landmarks)
                                        ? dump_max_landmarks
                                        : n;
                        pending_landmarks_xyz.reserve((size_t)keep * 3);
                        for (long li = 0; li < keep; li++) {
                            pending_landmarks_xyz.push_back(lm.landmarks[li].x);
                            pending_landmarks_xyz.push_back(lm.landmarks[li].y);
                            pending_landmarks_xyz.push_back(lm.landmarks[li].z);
                        }
                    }
                    delete[] lm.landmarks;

                    XRSLAMQualifiedLandmarks qualified;
                    qualified.landmarks = nullptr;
                    qualified.num_landmarks = 0;
                    try {
                        XRSLAMGetResult(XRSLAM_RESULT_QUALIFIED_LANDMARKS,
                                        &qualified);
                    } catch (...) {
                        qualified.landmarks = nullptr;
                        qualified.num_landmarks = 0;
                    }
                    if (qualified.landmarks != nullptr &&
                        qualified.num_landmarks > 0) {
                        long n = qualified.num_landmarks;
                        long keep =
                            (dump_max_landmarks > 0 && n > dump_max_landmarks)
                                ? dump_max_landmarks
                                : n;
                        pending_qualified_landmarks.assign(
                            qualified.landmarks, qualified.landmarks + keep);
                    }
                    delete[] qualified.landmarks;
                }

                if (processed % 50 == 0)
                    fprintf(stderr, "... %ld frames processed\n", processed);
            } else {
                // No IMU seen yet on either channel: RunOneFrame was never
                // called for this image -- not a real processed frame.
                open_frame_idx = -1;
            }
        }
    }
    flush_pending();

    if (dump_jsonl_stream.is_open()) {
        dump_jsonl_stream.close();
        fprintf(stderr, "wrote %s (%ld rows)\n", dump_jsonl_path.c_str(), open_frame_idx + 1);
    }

    XRSLAMDestroy();
    if (!generated_slam_cfg_path.empty())
        std::remove(generated_slam_cfg_path.c_str());

    std::vector<double> sorted_ms = frame_times_ms;
    std::sort(sorted_ms.begin(), sorted_ms.end());
    double mean = 0;
    for (double v : sorted_ms)
        mean += v;
    mean = sorted_ms.empty() ? 0 : mean / (double)sorted_ms.size();
    double p50 = percentile(sorted_ms, 0.50);
    double p95 = percentile(sorted_ms, 0.95);
    double max_ms = sorted_ms.empty() ? 0 : sorted_ms.back();

    long n_states = state_counts[0] + state_counts[1] + state_counts[2];

    std::ostringstream j;
    j << "{\n";
    j << "  \"n_frames\": " << processed << ",\n";
    j << "  \"n_frames_extracted\": " << frames.size() << ",\n";
    j << "  \"n_imu_samples\": " << imu.size() << ",\n";
    j << "  \"states\": {\n";
    j << "    \"INITIALIZING\": " << state_counts[0] << ",\n";
    j << "    \"TRACKING_SUCCESS\": " << state_counts[1] << ",\n";
    j << "    \"TRACKING_FAIL\": " << state_counts[2] << "\n";
    j << "  },\n";
    j << "  \"states_pct\": {\n";
    j << std::fixed << std::setprecision(3);
    j << "    \"INITIALIZING\": " << (n_states ? 100.0 * state_counts[0] / n_states : 0.0)
      << ",\n";
    j << "    \"TRACKING_SUCCESS\": "
      << (n_states ? 100.0 * state_counts[1] / n_states : 0.0) << ",\n";
    j << "    \"TRACKING_FAIL\": " << (n_states ? 100.0 * state_counts[2] / n_states : 0.0)
      << "\n";
    j << "  },\n";
    j << "  \"latency_ms\": {\n";
    j << std::setprecision(4);
    j << "    \"mean\": " << mean << ",\n";
    j << "    \"p50\": " << p50 << ",\n";
    j << "    \"p95\": " << p95 << ",\n";
    j << "    \"max\": " << max_ms << "\n";
    j << "  },\n";
    j << "  \"trajectory\": [\n";
    for (size_t k = 0; k < trajectory.size(); k++) {
        const auto &r = trajectory[k];
        j << "    {\"t\": " << std::setprecision(9) << (double)(r.t_ns - epoch_ns) / 1e9
          << ", \"state\": " << r.state << ", \"p\": [" << std::setprecision(6) << r.p[0]
          << ", " << r.p[1] << ", " << r.p[2] << "], \"q\": [" << r.q[0] << ", " << r.q[1]
          << ", " << r.q[2] << ", " << r.q[3] << "], \"frame_ms\": " << std::setprecision(4)
          << r.frame_ms << "}";
        if (k + 1 < trajectory.size())
            j << ",";
        j << "\n";
    }
    j << "  ]\n";
    j << "}\n";

    if (!out_path.empty()) {
        std::ofstream of(out_path);
        of << j.str();
        of.close();
        fprintf(stderr, "wrote %s\n", out_path.c_str());
    } else {
        printf("%s", j.str().c_str());
    }

    return 0;
}
