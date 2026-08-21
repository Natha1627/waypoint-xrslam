#ifndef XRSLAM_INSPECTOR_H
#define XRSLAM_INSPECTOR_H

#include <any>
#include <cstdint>
#include <mutex>
#include <xrslam/xrslam.h>

namespace xrslam {

class Track;

using point2i = vector<2, false, int>;
using color3b = vector<3, false, unsigned char>;

struct KeyframeState {
    double t;
    quaternion q;
    vector<3> p;
    vector<3> v;
    vector<3> bg;
    vector<3> ba;
};

struct Landmark {
    vector<3> p;
    uint64_t track_id = 0;
    bool valid = false;
    bool triangulated = false;
    bool outlier = false;
    bool static_track = false;
    size_t observation_count = 0;
    size_t life = 0;
    double inverse_depth = 0.0;
    double triangulation_angle_rad = 0.0;
    double mean_reprojection_error_px = 0.0;
    double max_reprojection_error_px = 0.0;
    vector<2> last_observation_px = vector<2>::Zero();
    double first_observation_timestamp = 0.0;
    double last_observation_timestamp = 0.0;
};

Landmark make_inspection_landmark(const Track &track);

class InspectPainter {
  public:
    virtual ~InspectPainter() = default;
    virtual void set_image(const Image *image) = 0;
    virtual void point(const point2i &p, const color3b &c, int size = 1,
                       int style = 0) {}
    virtual void line(const point2i &p1, const point2i &p2, const color3b &c,
                      int thickness = 1) {}
};

class InspectionSupport {
    struct VersionTag;

  public:
    enum InspectItem {
        RESERVED = 0,
        input_data_fps,
        input_real_fps,
        input_output_lag,
        feature_tracker_angle_misalignment,
        feature_tracker_painter,
        sliding_window_track_painter,
        sliding_window_reprojection_painter,
        sliding_window_landmarks,
        sliding_window_current_bg,
        sliding_window_current_ba,
        feature_tracker_time,
        bundle_adjustor_solve_time,
        bundle_adjustor_marginalization_time,
        ITEM_COUNT
    };

    InspectionSupport(const VersionTag &tag);
    ~InspectionSupport();

    static std::pair<std::any &, std::unique_lock<std::mutex>>
    get(InspectItem item);

  private:
    static InspectionSupport &support();
    std::vector<std::pair<std::any, std::mutex>> storage;
};

} // namespace xrslam

#if defined(XRSLAM_ENABLE_DEBUG_INSPECTION)
#define inspect_debug(item, var)                                               \
    if constexpr (auto [var, var##_lock] = ::xrslam::InspectionSupport::get(   \
                      ::xrslam::InspectionSupport::item);                      \
                  true)
#else
#define inspect_debug(item, var) if constexpr (std::any var; false)
#endif

#define inspect(item, var)                                                     \
    if constexpr (auto [var, var##_lock] = ::xrslam::InspectionSupport::get(   \
                      ::xrslam::InspectionSupport::item);                      \
                  true)

#endif // XRSLAM_INSPECTOR_H
