#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "detector.hpp"
#include "capsule.hpp"
#include "joint_load.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <unordered_map>

int main() {
  
  const std::string urdf_path = "../data/fr3_capsule.urdf";
  const std::string capsule_json_path = "../data/capsule_config.json";

  // 1. Load robot model from URDF
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);
  pinocchio::Data data(model);

  // 2. Load capsules from JSON
  CapsuleCollisionDetector detector(capsule_json_path);
  const std::vector<Capsule>& capsules = detector.get_capsules();
  // 3. Load the joint_pose from JSON
  const std::string joint_json_path = "../data/joint_value.json";
  Eigen::VectorXd q = load_joint_values_from_json(joint_json_path, model.nq);


  // 4. Perform FK and update world poses
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

 std::unordered_map<std::string, Eigen::Isometry3d> link_poses;
  for (const auto& capsule : capsules) {
  pinocchio::FrameIndex idx = model.getFrameId(capsule.link_name);
  if (idx == model.nframes) {
    std::cerr << "Frame not found: " << capsule.link_name << std::endl;
    continue;
  }
  const auto& M = data.oMf[idx];  // Pinocchio SE3
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = M.rotation();
  iso.translation() = M.translation();
  link_poses[capsule.link_name] = iso;
}

  // 5. Update capsule world poses
  detector.update_poses(link_poses);

  // 6. Check for collisions
  if (detector.has_collision()) {
    std::cout << "Collision detected!" << std::endl;
    for (const auto& pair : detector.get_colliding_pairs()) {
      std::cout << "Capsule " << pair.first << " collides with " << pair.second << std::endl;
    }
  } else {
    std::cout << "[✓] No collision detected." << std::endl;
  }

  return 0;
}