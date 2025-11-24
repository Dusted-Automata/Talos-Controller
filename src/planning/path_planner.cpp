#include "path_planner.hpp"
#include "cppmap3d.hh"
#include "math.hpp"
#include "types.hpp"
#include <iostream>

// inline double
// dist(const Pose &a, const Pose &b)
// {
//     double x = std::pow((b.local_point.east() - a.local_point.east()), 2);
//     double y = std::pow((b.local_point.north() - a.local_point.north()), 2);
//     double z = std::pow((b.local_point.up() - a.local_point.up()), 2);
//     double val = std::sqrt(x + y + z);
//     return val;
// };
//

size_t
Path_Planner::re_identify_position(ENU position) {
    Path* path = global_cursor->path;
    const size_t N = path ? path->num_waypoints() : 0;

    if (N <= 1) {
        global_cursor->next_waypoint = 0;
        global_cursor->update_target_stop();
        return 0;
    }

    f64 best_dist = std::numeric_limits<f64>::infinity();
    size_t best_seg_end_idx = 1;
    f64 best_t = 0.0;

    // TODO: Change from local_point to Ecef points
    for (size_t i = 1; i < N; ++i) {
        const ENU a = path->waypoint(i - 1).local_point;
        const ENU b = path->waypoint(i).local_point;
        const Eigen::Vector3d s = (b - a); // segment vector
        const f64 ss = s.squaredNorm();
        if (ss <= 1e-12) continue; // degenerate segment

        const Eigen::Vector3d ap = position - a;
        f64 t = ap.dot(s) / ss;             // un-clamped projection
        f64 t_clamped = std::min<f64>(1.0, std::max<f64>(0.0, t));
        const Eigen::Vector3d proj = a.raw() + t_clamped * s;
        const f64 distance = (position - proj).raw().norm();

        if (distance < best_dist) {
            best_dist = distance;
            best_seg_end_idx = i;
            best_t = t_clamped;
        }
    }

    size_t waypoint_idx = (best_t >= 0.5) ? best_seg_end_idx : (best_seg_end_idx - 1);

    if (path->is_looping()) {
        const ENU a = path->waypoint(N - 1).local_point;
        const ENU b = path->waypoint(0).local_point;
        const Eigen::Vector3d s = (b - a); // segment vector
        const f64 ss = s.squaredNorm();
        if (!(ss <= 1e-12)) {
            const Eigen::Vector3d ap = position - a;
            f64 t = ap.dot(s) / ss;             // un-clamped projection
            f64 t_clamped = std::min<f64>(1.0, std::max<f64>(0.0, t));
            const Eigen::Vector3d proj = a.raw() + t_clamped * s;
            const f64 distance = (position - proj).raw().norm();

            if (distance < best_dist) {
                best_dist = distance;
                best_seg_end_idx = 0;
                best_t = t_clamped;
            }
        }

        waypoint_idx = (best_t >= 0.5) ? 0 : (best_seg_end_idx);
    }

    // if (best_t > 0.5) {
    //     waypoint_idx = facing_forward ? best_seg_end_idx : (best_seg_end_idx - 1);
    // } else if (best_t < 0.5) {
    //     waypoint_idx = facing_forward ? (best_seg_end_idx - 1) : best_seg_end_idx;
    // } else {
    //     // Exactly mid-segment: use heading
    //     waypoint_idx = facing_forward ? best_seg_end_idx : (best_seg_end_idx - 1);
    // }
    printf("total: %zu | closest_distance: %f | seg_end_idx: %zu | t: %.3f | next_wp: %zu\n",
           N, best_dist, best_seg_end_idx, best_t, waypoint_idx);

    global_cursor->next_waypoint = best_seg_end_idx;
    global_cursor->update_target_stop();
    return waypoint_idx;
}
