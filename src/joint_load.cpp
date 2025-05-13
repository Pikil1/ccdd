#include "joint_load.hpp"
#include <fstream>
#include <stdexcept>

Eigen::VectorXd load_joint_values_from_json(const std::string& path, int nq) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open joint JSON file: " + path);
  }

  nlohmann::json j;
  file >> j;

  if (!j.contains("joint_values") || !j["joint_values"].is_array()) {
    throw std::runtime_error("Invalid JSON format: expected key 'joint_values'.");
  }

  auto arr = j["joint_values"];
  if (arr.size() < nq) {
    throw std::runtime_error("Not enough joint values in JSON.");
  }

  Eigen::VectorXd q(nq);
  for (int i = 0; i < nq; ++i) {
    q[i] = arr[i];
  }
  return q;
}