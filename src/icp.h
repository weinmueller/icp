#pragma once

#include <Eigen/Dense>
#include <vector>

enum class NNMethod {
    BruteForce,
    KDTree,
};

enum class ICPMethod {
    PointToPoint,   // SVD-based (Besl & McKay)
    PointToPlane,   // minimizes (R*s + t - t_i) . n_i
    PlaneToPlane,   // symmetric: uses normals from both clouds
};

struct ICPSettings {
    bool rotation = true;
    bool translation = true;
    bool scaling = false;
    NNMethod nn_method = NNMethod::KDTree;
    ICPMethod method = ICPMethod::PointToPoint;
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
              const ICPSettings& settings = {},
              const std::vector<Eigen::Vector3d>& source_normals = {},
              const std::vector<Eigen::Vector3d>& target_normals = {});
