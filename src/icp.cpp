#include "icp.h"
#include <limits>

static std::vector<int> find_closest_points(
    const std::vector<Eigen::Vector3d>& src,
    const std::vector<Eigen::Vector3d>& tgt)
{
    std::vector<int> indices(src.size());
    for (size_t i = 0; i < src.size(); ++i) {
        double best_dist = std::numeric_limits<double>::max();
        int best_idx = 0;
        for (size_t j = 0; j < tgt.size(); ++j) {
            double d = (src[i] - tgt[j]).squaredNorm();
            if (d < best_dist) {
                best_dist = d;
                best_idx = static_cast<int>(j);
            }
        }
        indices[i] = best_idx;
    }
    return indices;
}

static void compute_transform(
    const std::vector<Eigen::Vector3d>& src,
    const std::vector<Eigen::Vector3d>& tgt,
    const std::vector<int>& correspondences,
    const ICPSettings& settings,
    Eigen::Matrix3d& R, Eigen::Vector3d& t, double& s)
{
    const size_t n = src.size();
    R = Eigen::Matrix3d::Identity();
    t = Eigen::Vector3d::Zero();
    s = 1.0;

    Eigen::Vector3d centroid_src = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid_tgt = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < n; ++i) {
        centroid_src += src[i];
        centroid_tgt += tgt[correspondences[i]];
    }
    centroid_src /= static_cast<double>(n);
    centroid_tgt /= static_cast<double>(n);

    if (settings.rotation) {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < n; ++i) {
            H += (src[i] - centroid_src) * (tgt[correspondences[i]] - centroid_tgt).transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixV() * svd.matrixU().transpose();

        if (R.determinant() < 0) {
            Eigen::Matrix3d V = svd.matrixV();
            V.col(2) *= -1;
            R = V * svd.matrixU().transpose();
        }

        if (settings.scaling) {
            double num = 0.0, den = 0.0;
            for (size_t i = 0; i < n; ++i) {
                Eigen::Vector3d d_tgt = tgt[correspondences[i]] - centroid_tgt;
                Eigen::Vector3d d_src = src[i] - centroid_src;
                num += d_tgt.dot(R * d_src);
                den += d_src.squaredNorm();
            }
            if (den > 0) s = num / den;
        }
    }

    if (settings.translation) {
        t = centroid_tgt - s * R * centroid_src;
    }
}

ICPResult icp(const std::vector<Eigen::Vector3d>& source,
              const std::vector<Eigen::Vector3d>& target,
              const ICPSettings& settings)
{
    ICPResult result;
    std::vector<Eigen::Vector3d> current = source;

    double prev_error = std::numeric_limits<double>::max();

    for (int iter = 0; iter < settings.max_iterations; ++iter) {
        auto correspondences = find_closest_points(current, target);

        double error = 0.0;
        for (size_t i = 0; i < current.size(); ++i)
            error += (current[i] - target[correspondences[i]]).squaredNorm();
        error /= static_cast<double>(current.size());

        if (std::abs(prev_error - error) < settings.tolerance) {
            result.error = error;
            result.iterations = iter;
            break;
        }
        prev_error = error;

        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        double s;
        compute_transform(current, target, correspondences, settings, R, t, s);

        // Accumulate transform
        result.rotation = R * result.rotation;
        result.translation = s * R * result.translation + t;
        result.scale *= s;

        for (auto& p : current)
            p = s * R * p + t;

        result.iterations = iter + 1;
        result.error = error;
    }

    return result;
}
