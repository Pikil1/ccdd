#pragma once

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <string>


Eigen::VectorXd load_joint_values_from_json(const std::string& path, int nq);