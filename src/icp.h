#pragma once

#include <Eigen/Dense>
#include <vector>

struct ICPSettings {
    bool rotation = true;
    bool translation = true;
    bool scaling = false;
    int max_iterations = 50;
    double tolerance = 1e-6;
};

struct ICPResult {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    double scale = 1.0;
    int iterations = 0;
    double error = 0.0;
};

ICPResult icp(const std::vector<Eigen::Vector3d>& source,
              const std::vector<Eigen::Vector3d>& target,
              const ICPSettings& settings = {});
