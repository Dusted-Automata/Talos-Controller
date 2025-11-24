#pragma once
#include "types.hpp"

enum class Direction : int { FORWARD = +1, REVERSE = -1 };
enum class Advance_Result { OK, REACHED_END, WRAPPED, EMPTY };


constexpr const char* to_string(Advance_Result c) {
    switch (c) {
        case Advance_Result::OK:   return "OK";
        case Advance_Result::REACHED_END:  return "Reached End";
        case Advance_Result::WRAPPED:   return "Wrapped";
        case Advance_Result::EMPTY:   return "empty";
        default: return "Other enum for Advance_Result";
    }
}


class Path {
public:
    Path() = default;

    void add_waypoint(const Pose& pose, bool stop = false);
    size_t num_waypoints() const;
    size_t num_segments() const; 
    size_t num_stops() const;
    size_t get_stop_waypoint_index(size_t stop_number) const;


    // TODO: wrap? 
    // TODO: figure out a different way to do this
    std::optional<size_t> convert_waypoint_to_stop_index(size_t waypoint_idx) const;

    // What are the differences between these? 
    // ======================================
    // this is unoptimal
    std::optional<size_t> next_stop_waypoint(size_t current_idx, Direction dir) const;
    std::optional<size_t> next_stop_index(size_t stop_idx, Direction dir) const;
    // ======================================

    const Pose& waypoint(size_t idx) const;
    bool is_stop(size_t idx) const;

    f64 segment_length(size_t from_idx) const;

    f64 distance_between(size_t from_idx, size_t to_idx, Direction dir) const;
    f64 total_length() const;
    void set_looping(bool loop);
    bool is_looping() const;
    void update_distances();
    f64 calculate_distance(const Pose& a, const Pose& b) const;

    std::vector<double> cumulative_distance;
    std::vector<Pose> waypoints;
private:
    std::vector<size_t> stop_indices;
    std::vector<bool> stops;
    f64 loop_closure_distance_{ 0.0 };
    bool looping = false;
};
 
std::vector<std::uint8_t> serializePoses(const std::vector<Pose>& poses);
inline void appendUint32(std::vector<std::uint8_t>& buf, std::uint32_t value);
inline void appendDouble(std::vector<std::uint8_t>& buf, double value);


struct Path_Cursor {
    // Immediate navigation - which waypoint segment we're on
    size_t current_waypoint =  0 ; // Waypoint we're coming from
    size_t next_waypoint =  1 ;    // Waypoint we're heading to (immediate)
    f64 progress = 0.0;          // [0.0, 1.0] progress toward next_waypoint
    Direction dir = Direction::FORWARD;
    Path *path;
    bool target_latched = true;


    // Goal tracking - which stop we're ultimately heading toward
    size_t target_stop_waypoint{ 0 };         // Waypoint index of the target stop
    std::optional<size_t> target_stop_idx; // Stop number (0, 1, 2, ...)

    void initialize(Path *path, Direction direction = Direction::FORWARD);
    Pose get_next_waypoint() const;

    // Update which stop we're ultimately heading toward
    void update_target_stop();
    f64 distance_to_next_waypoint(const Pose &a) const;
    f64 distance_to_next_waypoint() const;

    // Distance to the TARGET STOP (ultimate destination)
    // TODO: Can cache the distances between waypoints and the targets
    f64 distance_to_target_stop() const;


    // Check if we've reached the target stop
    bool at_target_stop(f64 goal_tolerance) const;
    void clear_target_latch();
    void consume_target();

    Advance_Result advance(f64 ds);
    Advance_Result advance_waypoint();
    Advance_Result set_current_waypoint(size_t waypoint_idx);

    void reverse();
};

Path read_json_latlon(std::filesystem::path file_path);
