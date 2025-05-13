#pragma once
#include <string>
#include <Eigen/Geometry> 

struct Capsule {
  std::string link_name;
  Eigen::Vector3d from_local;
  Eigen::Vector3d to_local;
  double radius;
  Eigen::Vector3d from_world;
  Eigen::Vector3d to_world;

  int id;
};

// load JSON file
std::vector<Capsule> load_capsules_from_json(const std::string& path);

// Update capsule after FK
void update_capsule_world_pose(std::vector<Capsule>& capsules, const std::unordered_map<std::string, Eigen::Isometry3d>& link_poses);

static double segment_segment_distance( const Eigen::Vector3d& p1, const Eigen::Vector3d& q1, const Eigen::Vector3d& p2, const Eigen::Vector3d& q2);

double capsule_distance(const Capsule& a, const Capsule& b);

bool is_colliding(const Capsule& a, const Capsule& b, double margin=1e-4);


