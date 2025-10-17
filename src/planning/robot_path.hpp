#pragma once
#include "types.hpp"

struct Path_Pose {
    Ecef point;
    ENU local_point;
    f32 heading;
};

struct Path_Segment {
  size_t start_index;
  size_t end_index;
};

struct Path_Segment_Info {
  const Pose& start;
  const Pose& end;
  f64 length;
};


class Robot_Path
{

  private:
    std::vector<Pose> path;
    std::vector<i32> stoppings;
    std::vector<f64> distance_to_next_waypoint; // TODO: Any other way to do this?
    bool reverse_forward = false;

  public:
    size_t current_index = 0;
    size_t goal_index = 1;
    size_t stop_index = 0;
    Robot_Path() = default;

    bool path_looping = false;

    void add_waypoints(const std::vector<Pose> &waypoints);
    void add_waypoint(const Pose waypoint);
    Pose current();
    Pose next();
    Pose next_stop();
    bool progress(const Path_Direction &dir);
    bool read_json_latlon(std::filesystem::path file_path);
    size_t size();
    void reset();
    void print();

    f64 calculate_distance(int waypoint_index, int stop_index);

    Pose &operator[](size_t index);
    const Pose &operator[](size_t index) const;
};

enum class Direction : int { FORWARD = +1, REVERSE = -1 };
enum class Advance_Result { OK, REACHED_END, WRAPPED, EMPTY };


constexpr const char* to_string(Advance_Result c) {
    switch (c) {
        case Advance_Result::OK:   return "OK";
        case Advance_Result::REACHED_END:  return "Reached End";
        case Advance_Result::WRAPPED:   return "Wrapped";
        case Advance_Result::EMPTY:   return "empty";
    }
}


class Path {
public:
    Path() = default;

    void add_waypoint(const Pose& pose, bool stop = false) {
        waypoints.push_back(pose);
        stops.push_back(stop);
        if (stop) {
            stop_indices.push_back(waypoints.size() - 1);
        }
        update_distances();
    }

    size_t num_waypoints() const { return waypoints.size(); }

    size_t num_segments() const {
        if (waypoints.size() < 2) return 0;
        if (is_loop_) {
            return waypoints.size();
        } else {
            return waypoints.size() - 1;
        }
    }

    size_t num_stops() const { return stop_indices.size(); }
    size_t get_stop_waypoint_index(size_t stop_number) const { 
        return stop_indices[stop_number]; 
    }


    // TODO: wrap? 
    // TODO: figure out a different way to do this
    std::optional<size_t> convert_waypoint_to_stop_index(size_t waypoint_idx) const {
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
    next_stop_waypoint(size_t current_idx, Direction dir) const
    {
        if (stop_indices.empty()) return std::nullopt;

        if (dir == Direction::FORWARD) {
            // Find first stop after current_idx
            for (size_t stop_wp_idx : stop_indices) {
                if (stop_wp_idx > current_idx) return stop_wp_idx;
            }
            // If looping, wrap to first stop
            if (is_loop_) return stop_indices.front();

        } else { // REVERSE
            // Find first stop before current_idx (search backward)
            for (auto it = stop_indices.rbegin(); it != stop_indices.rend(); ++it) {
                if (*it < current_idx) return *it;
            }
            // If looping, wrap to last stop
            if (is_loop_) return stop_indices.back();
        }

        printf("next_stop_waypoint GONNA RETURN NULLOPT\n");
        return std::nullopt;
    }

    std::optional<size_t>
    next_stop_index(size_t stop_idx, Direction dir) const
    {
        if (waypoints.empty()) return std::nullopt;

        if (dir == Direction::FORWARD) {
            stop_idx = (stop_idx+ 1) % stop_indices.size();
            if (!is_loop_ && stop_idx == 0) return std::nullopt; // Reached end
            return stop_idx;
        } else {
            if (stop_idx == 0) {
                if (!is_loop_) return std::nullopt; // Reached start
                return stop_indices.size() - 1;
            } else {
                return --stop_idx;
            }
        }
        return std::nullopt; // No stops found
    }


    // ======================================

    const Pose& waypoint(size_t idx) const { return waypoints[idx]; }
    bool is_stop(size_t idx) const { return stops[idx]; }

    f64
    segment_length(size_t from_idx) const
    {
        if (from_idx >= waypoints.size()) return 0.0;

        if (is_loop_ && from_idx == waypoints.size() - 1) {
            return loop_closure_distance_; // Last -> First segment
        }

        if (from_idx >= waypoints.size() - 1) return 0.0;
        f64 distance = cumulative_distance[from_idx + 1] - cumulative_distance[from_idx];
        return distance;
    }

    f64
    distance_between(size_t from_idx, size_t to_idx, Direction dir) const
    {
        if (from_idx == to_idx) return 0.0;

        if (dir == Direction::FORWARD) {
            if (to_idx > from_idx) {
                return cumulative_distance[to_idx] - cumulative_distance[from_idx];
            } else if (is_loop_) {
                // Wrapped around: from -> end -> start -> to
                return (cumulative_distance.back() - cumulative_distance[from_idx]) + loop_closure_distance_
                + cumulative_distance[to_idx];
            }
        } else { // REVERSE
            if (to_idx < from_idx) {
                return cumulative_distance[from_idx] - cumulative_distance[to_idx];
            } else if (is_loop_) {
                // Wrapped backward: from -> start -> end -> to
                return cumulative_distance[from_idx] + loop_closure_distance_
                + (cumulative_distance.back() - cumulative_distance[to_idx]);
            }
        }

        return 0.0; // Invalid path
    }
    f64
    total_length() const
    {
        if (waypoints.size() < 2) return 0.0;
        f64 len = cumulative_distance.back();
        if (is_loop_) len += loop_closure_distance_;
        return len;
    }

    void set_looping(bool loop) {
        is_loop_ = loop;
        update_distances();
    }

    bool is_looping() const { return is_loop_; }


    void
    update_distances()
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
        if (is_loop_ && waypoints.size() >= 2) {
            loop_closure_distance_ = calculate_distance(waypoints.back(), waypoints.front());
        } else {
            loop_closure_distance_ = 0.0;
        }
    }

    f64 calculate_distance(const Pose& a, const Pose& b) const {
        return (a.local_point - b.local_point).norm();
    }

    std::vector<double> cumulative_distance;
private:
    std::vector<size_t> stop_indices;
    std::vector<Pose> waypoints;
    std::vector<bool> stops;
    f64 loop_closure_distance_{ 0.0 };
    bool is_loop_{ false };
};

struct Path_Cursor {
    // Immediate navigation - which waypoint segment we're on
    size_t current_waypoint =  0 ; // Waypoint we're coming from
    size_t next_waypoint =  1 ;    // Waypoint we're heading to (immediate)
    f64 progress = 0.0;          // [0.0, 1.0] progress toward next_waypoint
    Direction dir = Direction::FORWARD;
    Path *path;

    // Goal tracking - which stop we're ultimately heading toward
    size_t target_stop_waypoint{ 0 };         // Waypoint index of the target stop
    std::optional<size_t> target_stop_idx; // Stop number (0, 1, 2, ...)

    void
    initialize(Path *path, Direction direction = Direction::FORWARD)
    {
        dir = direction;
        current_waypoint = 0;
        next_waypoint = dir == Direction::FORWARD ? 1 : 0;
        progress = 0.0;
        this->path = path;
        update_target_stop();
    }

    Pose
    get_next_waypoint() const
    {
        if (dir == Direction::FORWARD) {
            return path->waypoint(next_waypoint);
        } else {
            return path->waypoint(current_waypoint);
        }
    }

    // Update which stop we're ultimately heading toward
    void
    update_target_stop()
    {
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
    distance_to_next_waypoint(const Pose &a) const
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
    distance_to_next_waypoint() const
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
    distance_to_target_stop() const 
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
    bool at_target_stop(f64 goal_tolerance) const
    {
        f64 distance = path->distance_between(current_waypoint, target_stop_waypoint, dir);
        return distance < goal_tolerance ;
    }

    Advance_Result
    advance(f64 ds) 
    {
        if (path->num_waypoints() < 2) return Advance_Result::EMPTY;

        f64 remaining_distance_to_next_waypoint = ds;

        if (remaining_distance_to_next_waypoint > 0.3) { // TODO: add padding
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
    advance_waypoint()
    {
        if (dir == Direction::FORWARD) {
            current_waypoint = next_waypoint;
            next_waypoint++;

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
            next_waypoint = current_waypoint;

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
    reverse()
    {
        dir = (dir == Direction::FORWARD) ? Direction::REVERSE : Direction::FORWARD;
        std::swap(current_waypoint, next_waypoint);
        progress = 1.0 - progress;
    }
};

Path read_json_latlon(std::filesystem::path file_path);
