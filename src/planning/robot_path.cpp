#include "robot_path.hpp"
#include "cppmap3d.hh"
#include "json.hpp"
#include <fstream>
#include <iostream>

// void
// Robot_Path::print()
// {
//     std::cout << "------------" << std::endl;
//     // std::string msg = std::format("Index: C{} | G{} - Path_Looping : ", current_index, goal_index, path_looping);
//     // std::cout << msg << std::endl;
//     for (size_t i = 0; i < path.size(); i++) {
//         std::cout << path[i].point.raw().transpose() << " | " << path[i].local_point.raw().transpose() << std::endl;
//     }
//     std::cout << "-----------" << std::endl;
// }

inline void appendDouble(std::vector<std::uint8_t>& buf, double value)
{
    std::uint8_t bytes[sizeof(double)];
    std::memcpy(bytes, &value, sizeof(double));
    buf.insert(buf.end(), bytes, bytes + sizeof(double));
}

inline void appendUint32(std::vector<std::uint8_t>& buf, std::uint32_t value)
{
    // If you care about endianness, convert to network order here.
    std::uint8_t bytes[4];
    bytes[0] = static_cast<std::uint8_t>((value >> 24) & 0xFF);
    bytes[1] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
    bytes[2] = static_cast<std::uint8_t>((value >>  8) & 0xFF);
    bytes[3] = static_cast<std::uint8_t>((value      ) & 0xFF);
    buf.insert(buf.end(), bytes, bytes + 4);
}

std::vector<std::uint8_t> serializePoses(const std::vector<Pose>& poses)
{
    std::vector<std::uint8_t> buf;
    buf.reserve(4 + poses.size() * 7 * sizeof(double));

    // 1) write count
    appendUint32(buf, static_cast<std::uint32_t>(poses.size()));

    // 2) write each pose
    for (const auto& p : poses) {
        const auto& pt = p.point;
        // const auto& q  = p.rotation;
        const auto& q  = p.heading;

        appendDouble(buf, pt.x());
        appendDouble(buf, pt.y());
        appendDouble(buf, pt.z());

        // Quaternion: w, x, y, z (choose an order and KEEP IT CONSISTENT)
        appendDouble(buf, q);
        // appendDouble(buf, q.x());
        // appendDouble(buf, q.y());
        // appendDouble(buf, q.z());
        // appendDouble(buf, q.w());
    }

    return buf;
}


using json = nlohmann::json;

Path
read_json_latlon(std::filesystem::path file_path)
{
    Path path;
    std::ifstream file(file_path);

    if (!file) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return path;
    }
    json data = json::parse(file);
    LLH llh, llh_origin;

    // Get origin LLH, to be able to compute ENU differences
    auto point = data["points"].front();
    if (point.contains("lat")) {

        llh_origin.lat() = to_radian(point["lat"]);
        llh_origin.lon() = to_radian(point["lon"]);
        llh_origin.alt() = point["alt"];
        // double offset = egm96_compute_altitude_offset(llh.lat(), llh.lon());
        // llh_origin.alt() += offset;

        for (size_t i = 0; i < data["points"].size(); i++){
            point = data["points"][i];
            Pose pose;
            llh.lat() = to_radian(point["lat"]);
            llh.lon() = to_radian(point["lon"]);
            llh.alt() = point["alt"];
            // double offset = egm96_compute_altitude_offset(llh.lat(), llh.lon());
            // llh.alt() += offset;
            std::cout << "lat: " << llh.lat() << " long: " << llh.lon() << " alt: " << llh.alt() << std::endl;

            Ecef ecef = cppmap3d::geodetic2ecef(llh);
            pose.point = ecef;

            ENU local = cppmap3d::ecef2enu(ecef, llh_origin);
            pose.local_point = local;

            if (point["stop"] == true) {
                printf("stop at index: %zu\n", i);
                path.add_waypoint(pose, true);
            } else {
                path.add_waypoint(pose, false);
            }
            f64 bearing = point["bearing"];
            if (bearing) {
                pose.heading = bearing;
            } 
        }
    } 
    return path;
}


void Path::add_waypoint(const Pose& pose, bool stop) {
    waypoints.push_back(pose);
    stops.push_back(stop);
    if (stop) {
        stop_indices.push_back(waypoints.size() - 1);
    }
    update_distances();
}

size_t Path::num_waypoints() const { return waypoints.size(); }

size_t Path::num_segments() const {
    if (waypoints.size() < 2) return 0;
    if (looping) {
        return waypoints.size();
    } else {
        return waypoints.size() - 1;
    }
}

size_t Path::num_stops() const { return stop_indices.size(); }
size_t Path::get_stop_waypoint_index(size_t stop_number) const { 
    return stop_indices[stop_number]; 
}


// TODO: wrap? 
// TODO: figure out a different way to do this
std::optional<size_t> Path::convert_waypoint_to_stop_index(size_t waypoint_idx) const {
    for (size_t i = 0; i < stop_indices.size(); ++i) {
        if (stop_indices[i] == waypoint_idx)
        {
            return i;
        }
    }
    return std::nullopt;
}


// What are the differences between these? 
// ======================================

std::optional<size_t> // this is unoptimal
Path::next_stop_waypoint(size_t current_idx, Direction dir) const
{
    if (stop_indices.empty()) return std::nullopt;

    if (dir == Direction::FORWARD) {
        // Find first stop after current_idx
        for (size_t stop_wp_idx : stop_indices) {
            if (stop_wp_idx > current_idx) return stop_wp_idx;
        }
        // If looping, wrap to first stop
        if (looping) return stop_indices.front();

    } else { // REVERSE
        // Find first stop before current_idx (search backward)
        for (auto it = stop_indices.rbegin(); it != stop_indices.rend(); ++it) {
            if (*it < current_idx) return *it;
        }
        // If looping, wrap to last stop
        if (looping) return stop_indices.back();
    }

    printf("next_stop_waypoint GONNA RETURN NULLOPT\n");
    return std::nullopt;
}

std::optional<size_t>
Path::next_stop_index(size_t stop_idx, Direction dir) const
{
    if (waypoints.empty()) return std::nullopt;

    if (dir == Direction::FORWARD) {
        stop_idx = (stop_idx+ 1) % stop_indices.size();
        if (!looping && stop_idx == 0) return std::nullopt; // Reached end
        return stop_idx;
    } else {
        if (stop_idx == 0) {
            if (!looping) return std::nullopt; // Reached start
            return stop_indices.size() - 1;
        } else {
            return --stop_idx;
        }
    }
    return std::nullopt; // No stops found
}


// ======================================

const Pose& Path::waypoint(size_t idx) const { return waypoints[idx]; }
bool Path::is_stop(size_t idx) const { return stops[idx]; }

f64
Path::segment_length(size_t from_idx) const
{
    if (from_idx >= waypoints.size()) return 0.0;

    if (looping && from_idx == waypoints.size() - 1) {
        return loop_closure_distance_; // Last -> First segment
    }

    if (from_idx >= waypoints.size() - 1) return 0.0;
    f64 distance = cumulative_distance[from_idx + 1] - cumulative_distance[from_idx];
    return distance;
}

f64
Path::distance_between(size_t from_idx, size_t to_idx, Direction dir) const
{
    if (from_idx == to_idx) return 0.0;

    if (dir == Direction::FORWARD) {
        if (to_idx > from_idx) {
            return cumulative_distance[to_idx] - cumulative_distance[from_idx];
        } else if (looping) {
            // Wrapped around: from -> end -> start -> to
            return (cumulative_distance.back() - cumulative_distance[from_idx]) + loop_closure_distance_
            + cumulative_distance[to_idx];
        }
    } else { // REVERSE
        if (to_idx < from_idx) {
            return cumulative_distance[from_idx] - cumulative_distance[to_idx];
        } else if (looping) {
            // Wrapped backward: from -> start -> end -> to
            return cumulative_distance[from_idx] + loop_closure_distance_
            + (cumulative_distance.back() - cumulative_distance[to_idx]);
        }
    }

    return 0.0; // Invalid path
}
f64
Path::total_length() const
{
    if (waypoints.size() < 2) return 0.0;
    f64 len = cumulative_distance.back();
    if (looping) len += loop_closure_distance_;
    return len;
}

void Path::set_looping(bool loop) {
    looping = loop;
    update_distances();
}

bool Path::is_looping() const { return looping; }


void
Path::update_distances()
{
    if (waypoints.size() <= 1) {
        cumulative_distance = { 0.0 };
        loop_closure_distance_ = 0.0;
        return;
    }

    cumulative_distance.resize(waypoints.size());
    cumulative_distance[0] = 0.0;

    for (size_t i = 1; i < waypoints.size(); ++i) {
        f64 dist = calculate_distance(waypoints[i - 1], waypoints[i]);
        cumulative_distance[i] = cumulative_distance[i - 1] + dist;
    }

    // Compute loop closure distance
    if (looping && waypoints.size() >= 2) {
        loop_closure_distance_ = calculate_distance(waypoints.back(), waypoints.front());
    } else {
        loop_closure_distance_ = 0.0;
    }
}

f64 Path::calculate_distance(const Pose& a, const Pose& b) const {
    // TODO: check if ENU - ENU is the same distance as ECEF - ECEF
    // return (a.local_point - b.local_point).norm();
    return (a.point - b.point).norm(); 
}


void
Path_Cursor::initialize(Path *path, Direction direction)
{
    dir = direction;
    current_waypoint = 0;
    next_waypoint = dir == Direction::FORWARD ? 1 : 0;
    progress = 0.0;
    this->path = path;
    update_target_stop();
}

Pose
Path_Cursor::get_next_waypoint() const
{
    if (dir == Direction::FORWARD) {
        return path->waypoint(next_waypoint);
    } else {
        return path->waypoint(current_waypoint);
    }
}

// Update which stop we're ultimately heading toward
void
Path_Cursor::update_target_stop()
{
    printf("at update_target_stop\n");
    std::optional<size_t> stop_wp = path->next_stop_waypoint(current_waypoint, dir);
    // if (target_stop_idx.has_value()) {
    //     printf("target_stop_waypoint: %zu | target_stop_number: %lu\n", target_stop_waypoint, target_stop_idx.value());
    // }
    if (stop_wp.has_value()) {
        printf("stop_wp %zu \n", stop_wp.value());
    }
    if (stop_wp) {
        target_stop_waypoint = *stop_wp;
        target_stop_idx = path->convert_waypoint_to_stop_index(*stop_wp);
    } else {
        target_stop_waypoint = current_waypoint;
        target_stop_idx = std::nullopt;
    }
}

f64
Path_Cursor::distance_to_next_waypoint(const Pose &a) const
{
    if (dir == Direction::FORWARD) {
        f64 dist = path->calculate_distance(a, path->waypoint(next_waypoint));
        return dist;
    } else {
        f64 dist = path->calculate_distance(path->waypoint(next_waypoint), a);
        return dist;
    }
}

f64
Path_Cursor::distance_to_next_waypoint() const
{
    size_t waypoint;
    if (dir == Direction::FORWARD) {
        waypoint = current_waypoint;
    } else {
        waypoint = next_waypoint;
    }
    f64 seg_length = path->segment_length(waypoint);
    return seg_length * (1 - progress);
}

// Distance to the TARGET STOP (ultimate destination)
// TODO: Can cache the distances between waypoints and the targets
f64
Path_Cursor::distance_to_target_stop() const 
{
    if (!target_stop_idx) return 0.0;
    f64 dist = distance_to_next_waypoint();

    // Add distance from next_waypoint to target stop
    // (this includes all intermediate waypoints)
    size_t from = next_waypoint;
    dist += path->distance_between(from, target_stop_waypoint, dir);

    return dist;
}


// Check if we've reached the target stop
bool Path_Cursor::at_target_stop(f64 goal_tolerance) const
{
    if (!target_stop_idx) return false;
    if (target_latched)   return false;
    printf("at target stop\n");
    f64 distance = path->distance_between(current_waypoint, target_stop_waypoint, dir);
    return distance < goal_tolerance ;
}

void Path_Cursor::clear_target_latch() { target_latched = false; }
void Path_Cursor::consume_target() {
    printf("at consume target\n");
    target_latched = true;
    target_stop_idx.reset();
}

Advance_Result
Path_Cursor::advance(f64 ds) 
{
    if (path->num_waypoints() < 2) return Advance_Result::EMPTY;

    f64 remaining_distance_to_next_waypoint = ds;

    if (remaining_distance_to_next_waypoint > 0.35) { // TODO: add padding
        f64 seg_length = path->segment_length(dir == Direction::FORWARD ? current_waypoint : next_waypoint);
        progress = (seg_length - remaining_distance_to_next_waypoint) / seg_length;

        // if (path->is_stop(current_waypoint)) {
        //     update_target_stop();
        // }

    } else {
        Advance_Result result = advance_waypoint();
        printf("advancing waypoint result! %s\n", to_string(result));
        if (result != Advance_Result::OK) {
            return result;
        }
    }

    return Advance_Result::OK;
}

Advance_Result
Path_Cursor::advance_waypoint()
{
    if (dir == Direction::FORWARD) {
        current_waypoint = next_waypoint;
        next_waypoint++;

        if (next_waypoint >= path->num_waypoints()) {
            if (path->is_looping()) {
                next_waypoint = 0;
                progress = 0.0;
                clear_target_latch();
                return Advance_Result::WRAPPED;
            } else {
                next_waypoint = current_waypoint;
                progress = 0.0;
                clear_target_latch();
                return Advance_Result::REACHED_END;
            }
        }
    } else { // REVERSE
        next_waypoint = current_waypoint;

        if (current_waypoint == 0) {
            if (path->is_looping()) {
                current_waypoint = path->num_waypoints() - 1;
                progress = 0.0;
                clear_target_latch();
                return Advance_Result::WRAPPED;
            } else {
                progress = 0.0;
                clear_target_latch();
                return Advance_Result::REACHED_END;
            }
        } else {
            current_waypoint--;
        }
    }
    progress = 0.0;
    clear_target_latch();
    return Advance_Result::OK;
}

Advance_Result
Path_Cursor::set_current_waypoint(size_t waypoint_idx)
{
    if (dir == Direction::FORWARD) {
        current_waypoint = waypoint_idx;
        next_waypoint = current_waypoint + 1;

        if (next_waypoint >= path->num_waypoints()) {
            if (path->is_looping()) {
                next_waypoint = 0;
                return Advance_Result::WRAPPED;
            } else {
                next_waypoint = current_waypoint;
                return Advance_Result::REACHED_END;
            }
        }
    } else { // REVERSE
        next_waypoint = waypoint_idx;
        current_waypoint = waypoint_idx;

        if (current_waypoint == 0) {
            if (path->is_looping()) {
                current_waypoint = path->num_waypoints() - 1;
                return Advance_Result::WRAPPED;
            } else {
                return Advance_Result::REACHED_END;
            }
        } else {
            current_waypoint--;
        }
    }
    progress = 0.0;

    return Advance_Result::OK;
}

// Pose
// get_pose(const Path &path) const
// {
//     return interpolate(path.waypoint(current_waypoint), path.waypoint(next_waypoint), progress);
// }

void
Path_Cursor::reverse()
{
    dir = (dir == Direction::FORWARD) ? Direction::REVERSE : Direction::FORWARD;
    std::swap(current_waypoint, next_waypoint);
    progress = 1.0 - progress;
}
