#include "capsule.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
std::vector<Capsule> load_capsules_from_json(const std::string& path) {
    //create a vector structure to load the json file
    std::ifstream file;
    file.open(path);
    nlohmann::json j;
    file >> j;

    std::vector<Capsule> capsules;

    int id_counter = 0;
    for (const auto& item : j) {
        Capsule c;
        c.link_name = item["link"];
        c.link_id=item["id"];
        c.from_local = Eigen::Vector3d(
            item["from"][0], item["from"][1], item["from"][2]);
        c.to_local = Eigen::Vector3d(
            item["to"][0], item["to"][1], item["to"][2]);
        c.radius = item["radius"];
        c.id = id_counter++;
        capsules.push_back(c);
    }

    return capsules;
}

void update_capsule_world_pose(std::vector<Capsule>& capsules, const std::unordered_map<std::string, Eigen::Isometry3d>& link_poses){
    //Translate the capsule from body coordinate to the world coordinate
  
    for (auto& cap : capsules) {
        auto it = link_poses.find(cap.link_id);
    if (it == link_poses.end()) {
        std::cerr << "[ERROR] Pose not found for link: " << cap.link_id << std::endl;
        continue;
    }

        const auto& T = link_poses.at(cap.link_id);
        cap.from_world = T * cap.from_local;
        cap.to_world   = T * cap.to_local;
    }
}

static double segment_segment_distance( const Eigen::Vector3d& p1, const Eigen::Vector3d& q1, const Eigen::Vector3d& p2, const Eigen::Vector3d& q2){
   //D(s, t) = ‖(p1 + s*d1) - (p2 + t*d2)‖²
   //s∗=(b∗f−c∗e)/(a∗e−b²)
   //t∗=(a∗f−b∗c)/(a∗e−b²)
    Eigen::Vector3d d1 = q1 - p1;
    Eigen::Vector3d d2 = q2 - p2;
    Eigen::Vector3d r = p1 - p2;

    double a = d1.dot(d1);
    double b = d1.dot(d2);
    double c = d1.dot(r);
    double e = d2.dot(d2);
    double f = d2.dot(r);
    double denom = a * e - b * b;

    double s = (b * f - c * e) / denom;
    double t = (a * f - b * c) / denom;

    s = std::clamp(s, 0.0, 1.0);
    t = std::clamp(t, 0.0, 1.0);

    Eigen::Vector3d cp1 = p1 + s * d1;
    Eigen::Vector3d cp2 = p2 + t * d2;
    return (cp1 - cp2).norm();

}

double capsule_distance(const Capsule& a, const Capsule& b){
    // shortest distance between 2 capsule
    double d = segment_segment_distance(a.from_world, a.to_world,b.from_world, b.to_world);
     double dist= (d - a.radius - b.radius);
    return dist;
}

bool capsule_is_colliding(const Capsule& a, const Capsule& b, double margin) {
   double dist = capsule_distance(a, b);
    if (dist < margin) {
        std::cout << "[Collision Detected] "
                  << "Capsule \"" << a.link_id << "\" <-> \"" << b.link_id << "\", "
                  << "distance = " << dist << ", margin = " << margin << "\n";
        return true;
    }
    return false;
    }
