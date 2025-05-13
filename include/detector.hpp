#pragma once

#include "capsule.hpp"
#include <unordered_map>
#include <string>
#include <vector>
#include <set>
#include <utility> 

class CapsuleCollisionDetector {
public:
 
  CapsuleCollisionDetector(const std::string& capsule_json_path);
 
  void update_poses(const std::unordered_map<std::string, Eigen::Isometry3d>& link_poses);
 
  std::vector<std::pair<int, int>> get_colliding_pairs(double margin = 1e-4) const;

  bool has_collision(double margin = 1e-4) const;
  const std::vector<Capsule>& get_capsules() const;

private:
  std::vector<Capsule> capsules_;
  std::set<std::pair<std::string, std::string>> candidate_pairs_;

  
  bool should_check(const std::string& a, const std::string& b) const;
};