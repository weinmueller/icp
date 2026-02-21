#pragma once

#include "kdtree.h"
#include <Eigen/Dense>
#include <vector>

inline std::vector<Eigen::Vector3d> estimate_normals(
    const std::vector<Eigen::Vector3d>& points, int k = 10)
{
    KDTree tree(points);
    std::vector<Eigen::Vector3d> normals(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        auto neighbors = tree.k_nearest(points[i], k);

        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (int idx : neighbors)
            centroid += points[idx];
        centroid /= static_cast<double>(neighbors.size());

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (int idx : neighbors) {
            Eigen::Vector3d d = points[idx] - centroid;
            cov += d * d.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        // Eigenvector corresponding to smallest eigenvalue is the normal
        normals[i] = solver.eigenvectors().col(0).normalized();
    }

    return normals;
}
