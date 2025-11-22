#pragma once

#include "pid.hpp"
#include "robot.hpp"
#include "types.hpp"
#include "json.hpp"
#include <string>

using json = nlohmann::json;


bool load_config(Robot &robot, const std::string &path);
void validate_fields(const json &j, const std::vector<std::string> &fields, const std::string &struct_name);
void from_json(const json &j, Kinematic_Constraints &obj);
void from_json(const json &j, Path_Config &obj);
void from_json(const json &j, PIDGains &obj);
void from_json(const json &j, Robot_Config &obj);
