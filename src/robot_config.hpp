#pragma once

#include "json.hpp"
#include "pid.hpp"
#include "robot.hpp"
#include "types.hpp"
#include <stdexcept>
#include <string>

using nlohmann::json;

void from_json(const json &j, Robot_Config &obj);

bool
load_config(Robot &robot, const std::string &path)
{

    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open JSON file.");
        return false;
    }

    json j;
    file >> j;

    robot.config = j.get<Robot_Config>();
    return true;
}

void
validate_fields(const json &j, const std::vector<std::string> &fields, const std::string &struct_name = "")
{
    for (const auto &field : fields) {
        if (!j.contains(field)) {
            std::string available = "";
            for (auto &[key, value] : j.items()) {
                available += "'" + key + "' ";
            }
            std::string context = struct_name.empty() ? "" : " in " + struct_name;
            throw std::runtime_error("Missing field '" + field + "'" + context + ". Available: " + available);
        }
    }

    for (auto &[key, value] : j.items()) {
        if (std::find(fields.begin(), fields.end(), key) == fields.end()) {
            std::string expected = "";
            for (const auto &f : fields) {
                expected += f + ", ";
            }
            if (!expected.empty()) {
                expected = expected.substr(0, expected.length() - 2); // Remove last ", "
            }
            std::string context = struct_name.empty() ? "" : " in " + struct_name;
            throw std::runtime_error("Unknown field '" + key + "'" + context + ". Expected: " + expected);
        }
    }
}

#define VALIDATE_FIELDS(json, struct_name, ...) validate_fields(json, { __VA_ARGS__ }, struct_name)

void
from_json(const json &j, Kinematic_Constraints &obj)
{
    VALIDATE_FIELDS(j, "Kinematic_Constraints", "velocity_forward_max", "velocity_backward_max",
        "velocity_turning_left_max", "velocity_turning_right_max", "acceleration_max", "deceleration_max", "alpha_max",
        "jerk_max", "jerk_omega_max");

    // Extract values
    // obj.velocity_forward_max = j.at("v_max").get<double>();
    // obj.velocity_backward_max = j.at("v_min").get<double>();
    // obj.velocity_turning_left_max = j.at("omega_max").get<double>();
    // obj.velocity_turning_right_max = j.at("omega_min").get<double>();
    // obj.acceleration_max = j.at("a_max").get<double>();
    // obj.deceleration_max = j.at("a_min").get<double>();
    // obj.alpha_max = j.at("alpha_max").get<double>();
    // obj.jerk_max = j.at("j_max").get<double>();
    // obj.jerk_omega_max = j.at("j_omega_max").get<double>();

    obj.velocity_forward_max = j.at("velocity_forward_max").get<double>();
    obj.velocity_backward_max = j.at("velocity_backward_max").get<double>();
    obj.velocity_turning_left_max = j.at("velocity_turning_left_max").get<double>();
    obj.velocity_turning_right_max = j.at("velocity_turning_right_max").get<double>();
    obj.acceleration_max = j.at("acceleration_max").get<double>();
    obj.deceleration_max = j.at("deceleration_max").get<double>();
    obj.alpha_max = j.at("alpha_max").get<double>();
    obj.jerk_max = j.at("jerk_max").get<double>();
    obj.jerk_omega_max = j.at("jerk_omega_max").get<double>();

    if (obj.velocity_forward_max < 0) throw std::invalid_argument("v_max must be non-negative");
    if (obj.velocity_turning_left_max < 0) throw std::invalid_argument("omega_max must be non-negative");
}

void
from_json(const json &j, PIDGains &obj)
{
    VALIDATE_FIELDS(j, "PIDGains", "k_p", "k_i", "k_d", "output_min", "output_max", "integral_min", "integral_max");

    obj.k_p = j.at("k_p").get<double>();
    obj.k_i = j.at("k_i").get<double>();
    obj.k_d = j.at("k_d").get<double>();
    obj.output_min = j.at("output_min").get<double>();
    obj.output_max = j.at("output_max").get<double>();
    obj.integral_min = j.at("integral_min").get<double>();
    obj.integral_max = j.at("integral_max").get<double>();

    if (obj.output_max <= obj.output_min) {
        throw std::invalid_argument("output_max must be greater than output_min");
    }
    if (obj.integral_max <= obj.integral_min) {
        throw std::invalid_argument("integral_max must be greater than integral_min");
    }
}

void
from_json(const json &j, Robot_Config &obj)
{
    VALIDATE_FIELDS(j, "Robot_Config", "version", "control_loop_hz", "goal_tolerance_in_meters",
        "kinematic_constraints", "PID");

    obj.control_loop_hz = j.at("control_loop_hz").get<int>();
    if (obj.control_loop_hz <= 0) {
        throw std::invalid_argument("control_loop_hz must be positive");
    }

    obj.goal_tolerance_in_meters = j.at("goal_tolerance_in_meters").get<double>();

    if (obj.goal_tolerance_in_meters <= 0) {
        throw std::invalid_argument("goal_tolerance_in_meters must be positive");
    }

    try {
        obj.kinematic_constraints = j.at("kinematic_constraints").get<Kinematic_Constraints>();
    } catch (const std::exception &e) {
        throw std::runtime_error("Error in kinematic_constraints: " + std::string(e.what()));
    }

    VALIDATE_FIELDS(j.at("PID"), "PID", "linear_gains", "angular_gains");

    try {
        obj.linear_gains = j.at("PID").at("linear_gains").get<PIDGains>();
    } catch (const std::exception &e) {
        throw std::runtime_error("Error in linear PID: " + std::string(e.what()));
    }

    try {
        obj.angular_gains = j.at("PID").at("angular_gains").get<PIDGains>();
    } catch (const std::exception &e) {
        throw std::runtime_error("Error in angular PID: " + std::string(e.what()));
    }
}
