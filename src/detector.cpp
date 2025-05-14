#include "detector.hpp"

CapsuleCollisionDetector::CapsuleCollisionDetector(const std::string& capsule_json_path) {
  capsules_ = load_capsules_from_json(capsule_json_path);

  candidate_pairs_ = {
    {"fr3_link0_sc","hand_2"},
    {"fr3_link1_sc","hand_2"},
    {"fr3_link2_sc","hand_2"},
    {"fr3_link3_sc","hand_2"},
    {"fr3_link4_sc","hand_2"},
    {"fr3_link5_main_sc","hand_2"},
    {"fr3_link5_side_sc","hand_2"},

    {"hand_1", "fr3_link0_sc"},
    {"hand_1", "fr3_link1_sc"},
    {"hand_1", "fr3_link2_sc"},
    {"hand_1", "fr3_link3_sc"},
    {"hand_1", "fr3_link4_sc"},
    {"hand_1", "fr3_link5_main_sc"},
    {"hand_1", "fr3_link5_side_sc"},

    {"fr3_link6_sc", "fr3_link0_sc"},
    {"fr3_link6_sc", "fr3_link1_sc"},
    {"fr3_link6_sc", "fr3_link2_sc"},

    {"fr3_link7_main_sc", "fr3_link0_sc"},
    {"fr3_link7_main_sc", "fr3_link1_sc"},
    
  };
}



void CapsuleCollisionDetector::update_poses(const std::unordered_map<std::string, Eigen::Isometry3d>& link_poses)
{
  update_capsule_world_pose(capsules_, link_poses);
}


//check if the capsule-pair is in the check-list
bool CapsuleCollisionDetector::should_check(const std::string& a, const std::string& b) const {
  std::string x = a, y = b;
  if (x > y) std::swap(x, y);
  return candidate_pairs_.count({x, y}) > 0;
}


std::vector<std::pair<int, int>> CapsuleCollisionDetector::get_colliding_pairs(double margin) const {

  std::vector<std::pair<int, int>> results;

  for (const auto& a : capsules_) {
    for (const auto& b : capsules_) {
      if (a.id >= b.id) continue;
      if (!should_check(a.link_id, b.link_id)) continue;
      if (capsule_is_colliding(a, b, margin)) {
        results.emplace_back(a.id, b.id);
      }
    }
  }
  return results;
}


const std::vector<Capsule>& CapsuleCollisionDetector::get_capsules() const {
  return capsules_;
}

bool CapsuleCollisionDetector::has_collision(double margin) const {
    for (size_t i = 0; i < capsules_.size(); ++i) {
        for (size_t j = i + 1; j < capsules_.size(); ++j) {
            const Capsule& a = capsules_[i];
            const Capsule& b = capsules_[j];
            if (should_check(a.link_id, b.link_id)) {
                if (capsule_is_colliding(a, b, margin)) {
                    return true;
                }
            }
        }
    }
    return false;
}



