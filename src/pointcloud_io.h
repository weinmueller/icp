#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

// Load/save point clouds as simple text files (one "x y z" per line).
// Lines starting with '#' are treated as comments.

std::vector<Eigen::Vector3d> load_pointcloud(const std::string& path);
void save_pointcloud(const std::string& path,
                     const std::vector<Eigen::Vector3d>& points);
