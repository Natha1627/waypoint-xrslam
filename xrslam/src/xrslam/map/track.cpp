#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/inspection.h>
#include <xrslam/map/track.h>

#include <algorithm>
#include <cmath>

namespace xrslam {

namespace {

bool finite3(const vector<3> &v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) &&
           std::isfinite(v.z());
}

} // namespace

Track::Track() { tag(TT_STATIC) = true; }
Track::~Track() = default;

const vector<3> &Track::get_keypoint(Frame *frame) const {
    return frame->get_keypoint(keypoint_refs.at(frame));
}

void Track::add_keypoint(Frame *frame, size_t keypoint_index) {
    keypoint_refs[frame] = keypoint_index;
    frame->tracks[keypoint_index] = this;
    frame->reprojection_error_factors[keypoint_index] = Solver::create_reprojection_error_factor(frame, this);

    if (this->tag(TT_TRIANGULATED))
        m_life++;
    else
        m_life = 1;
}

void Track::remove_keypoint(Frame *frame, bool suicide_if_empty) {
    size_t keypoint_index = keypoint_refs.at(frame);
    std::optional<vector<3>> landmark;
    if (frame == first_frame()) {
        landmark = get_landmark_point();
    }
    frame->tracks[keypoint_index] = nullptr;
    frame->reprojection_error_factors[keypoint_index].reset();
    keypoint_refs.erase(frame);
    if (keypoint_refs.size() > 0) {
        if (landmark.has_value()) {
            set_landmark_point(landmark.value());
        }
    } else {
        tag(TT_VALID) = false;
        if (suicide_if_empty) {
            map->recycle_track(this);
        }
    }
}

std::optional<vector<3>> Track::triangulate() {
    std::vector<matrix<3, 4>> Ps;
    std::vector<vector<3>> ps;
    for (const auto &[frame, keypoint_index] : keypoint_map()) {
        matrix<3, 4> P;
        matrix<3, 3> R;
        vector<3> T;
        auto pose = frame->get_pose(frame->camera);
        R = pose.q.conjugate().matrix();
        T = -(R * pose.p);
        P << R, T;
        Ps.push_back(P);
        ps.push_back(frame->get_keypoint(keypoint_index));
    }

    bool is_valid = true;
    vector<4> hlandmark = triangulate_point(Ps, ps);
    for (size_t i = 0; i < ps.size(); ++i) {
        vector<3> qi = Ps[i] * hlandmark;
        if (!(qi[2] * hlandmark[3] > 0)) {
            is_valid = false;
            break;
        }
    }
    if (is_valid) {
        m_life = 1;
        return hlandmark.hnormalized();
    } else {
        return {};
    }
}

double Track::triangulation_angle(const vector<3> &p) const {
    vector<3> nref = (p - first_frame()->get_pose(first_frame()->camera).p)
                         .stableNormalized();
    double max_angle = 0;
    for (const auto &[frame, keypoint_index] : keypoint_map()) {
        auto pose = frame->get_pose(frame->camera);
        vector<3> n = (p - pose.p).stableNormalized();
        max_angle = std::max(max_angle, acos(n.dot(nref)));
    }
    return max_angle;
}

Landmark make_inspection_landmark(const Track &track) {
    Landmark result;
    result.track_id = static_cast<uint64_t>(track.id());
    result.valid = track.tag(TT_VALID);
    result.triangulated = track.tag(TT_TRIANGULATED);
    result.outlier = track.tag(TT_OUTLIER);
    result.static_track = track.tag(TT_STATIC);
    result.observation_count = track.keypoint_num();
    result.life = track.m_life;
    result.inverse_depth = track.landmark.inv_depth;

    if (track.keypoint_num() == 0) {
        result.p.setZero();
        return result;
    }

    result.p = track.get_landmark_point();
    const auto first = track.first_keypoint();
    const auto last = track.last_keypoint();
    if (first.first != nullptr && first.first->image != nullptr) {
        result.first_observation_timestamp = first.first->image->t;
    }
    if (last.first != nullptr && last.first->image != nullptr) {
        result.last_observation_timestamp = last.first->image->t;
    }
    if (last.first != nullptr) {
        result.last_observation_px =
            apply_k(last.first->get_keypoint(last.second), last.first->K);
    }

    if (!result.triangulated || !finite3(result.p)) {
        return result;
    }

    const PoseState first_camera = first.first->get_pose(first.first->camera);
    const vector<3> reference_ray =
        (result.p - first_camera.p).stableNormalized();
    double max_angle = 0.0;
    double reprojection_sum = 0.0;
    double reprojection_max = 0.0;
    size_t reprojection_count = 0;

    for (const auto &observation : track.keypoint_map()) {
        Frame *frame = observation.first;
        if (frame == nullptr) {
            continue;
        }
        const PoseState camera = frame->get_pose(frame->camera);
        const vector<3> world_ray =
            (result.p - camera.p).stableNormalized();
        const double dot = std::max(
            -1.0, std::min(1.0, reference_ray.dot(world_ray)));
        max_angle = std::max(max_angle, std::acos(dot));

        const vector<3> point_camera =
            camera.q.conjugate() * (result.p - camera.p);
        if (!finite3(point_camera) || point_camera.z() <= 1e-9) {
            continue;
        }
        const vector<2> predicted_px = apply_k(point_camera, frame->K);
        const vector<2> observed_px =
            apply_k(frame->get_keypoint(observation.second), frame->K);
        const double error_px = (predicted_px - observed_px).norm();
        if (!std::isfinite(error_px)) {
            continue;
        }
        reprojection_sum += error_px;
        reprojection_max = std::max(reprojection_max, error_px);
        ++reprojection_count;
    }

    result.triangulation_angle_rad = max_angle;
    if (reprojection_count > 0) {
        result.mean_reprojection_error_px =
            reprojection_sum / static_cast<double>(reprojection_count);
        result.max_reprojection_error_px = reprojection_max;
    }
    return result;
}

vector<3> Track::get_landmark_point() const {
    const auto &[frame, keypoint_index] = first_keypoint();
    auto camera = frame->get_pose(frame->camera);
    return camera.q * frame->get_keypoint(keypoint_index) / landmark.inv_depth +
           camera.p;
}

void Track::set_landmark_point(const vector<3> &p) {
    const auto [frame, keypoint_index] = first_keypoint();
    auto camera = frame->get_pose(frame->camera);
    landmark.inv_depth = 1.0 / (camera.q.conjugate() * (p - camera.p)).norm();
}

} // namespace xrslam
