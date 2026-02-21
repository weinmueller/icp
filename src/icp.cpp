#include "icp.h"
#include "kdtree.h"
#include "normals.h"
#include <cmath>
#include <limits>

static std::vector<int> find_closest_brute(
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

static std::vector<int> find_closest_kdtree(
    const std::vector<Eigen::Vector3d>& src,
    const std::vector<Eigen::Vector3d>& tgt)
{
    KDTree tree(tgt);
    std::vector<int> indices(src.size());
    for (size_t i = 0; i < src.size(); ++i)
        indices[i] = tree.nearest(src[i]);
    return indices;
}

static std::vector<int> find_closest_points(
    const std::vector<Eigen::Vector3d>& src,
    const std::vector<Eigen::Vector3d>& tgt,
    NNMethod method)
{
    switch (method) {
        case NNMethod::KDTree:     return find_closest_kdtree(src, tgt);
        case NNMethod::BruteForce: return find_closest_brute(src, tgt);
    }
    return find_closest_brute(src, tgt);
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

// Point-to-plane: linearized least-squares using small-angle approximation.
// Solves for x = [rx, ry, rz, tx, ty, tz] minimizing sum_i ((R*si + t - ti) . ni)^2
static void compute_transform_point_to_plane(
    const std::vector<Eigen::Vector3d>& src,
    const std::vector<Eigen::Vector3d>& tgt,
    const std::vector<int>& correspondences,
    const std::vector<Eigen::Vector3d>& tgt_normals,
    Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    const size_t n = src.size();
    R = Eigen::Matrix3d::Identity();
    t = Eigen::Vector3d::Zero();

    Eigen::Matrix<double, 6, 6> ATA = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> ATb = Eigen::Matrix<double, 6, 1>::Zero();

    for (size_t i = 0; i < n; ++i) {
        const Eigen::Vector3d& s = src[i];
        const Eigen::Vector3d& ti = tgt[correspondences[i]];
        const Eigen::Vector3d& ni = tgt_normals[correspondences[i]];

        // a = [s x n, n] (cross product of source point with normal, then normal)
        Eigen::Matrix<double, 6, 1> a;
        a.head<3>() = s.cross(ni);
        a.tail<3>() = ni;

        double b = (ti - s).dot(ni);

        ATA += a * a.transpose();
        ATb += a * b;
    }

    Eigen::Matrix<double, 6, 1> x = ATA.ldlt().solve(ATb);

    double rx = x(0), ry = x(1), rz = x(2);
    t = x.tail<3>();

    // Small-angle rotation matrix
    R << 1.0,  -rz,   ry,
         rz,   1.0,  -rx,
        -ry,   rx,   1.0;

    // Re-orthogonalize via SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) {
        Eigen::Matrix3d U = svd.matrixU();
        U.col(2) *= -1;
        R = U * svd.matrixV().transpose();
    }
}

// Plane-to-plane (symmetric ICP): uses normals from both clouds.
// Combined normal n_i = n_source_i + n_target_i
static void compute_transform_plane_to_plane(
    const std::vector<Eigen::Vector3d>& src,
    const std::vector<Eigen::Vector3d>& tgt,
    const std::vector<int>& correspondences,
    const std::vector<Eigen::Vector3d>& src_normals,
    const std::vector<Eigen::Vector3d>& tgt_normals,
    Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    const size_t n = src.size();
    R = Eigen::Matrix3d::Identity();
    t = Eigen::Vector3d::Zero();

    Eigen::Matrix<double, 6, 6> ATA = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> ATb = Eigen::Matrix<double, 6, 1>::Zero();

    for (size_t i = 0; i < n; ++i) {
        const Eigen::Vector3d& si = src[i];
        const Eigen::Vector3d& ti = tgt[correspondences[i]];
        const Eigen::Vector3d& ns = src_normals[i];
        const Eigen::Vector3d& nt = tgt_normals[correspondences[i]];

        Eigen::Vector3d ni = (ns + nt).normalized();

        Eigen::Matrix<double, 6, 1> a;
        a.head<3>() = si.cross(ni);
        a.tail<3>() = ni;

        double b = (ti - si).dot(ni);

        ATA += a * a.transpose();
        ATb += a * b;
    }

    Eigen::Matrix<double, 6, 1> x = ATA.ldlt().solve(ATb);

    double rx = x(0), ry = x(1), rz = x(2);
    t = x.tail<3>();

    R << 1.0,  -rz,   ry,
         rz,   1.0,  -rx,
        -ry,   rx,   1.0;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) {
        Eigen::Matrix3d U = svd.matrixU();
        U.col(2) *= -1;
        R = U * svd.matrixV().transpose();
    }
}

ICPResult icp(const std::vector<Eigen::Vector3d>& source,
              const std::vector<Eigen::Vector3d>& target,
              const ICPSettings& settings,
              const std::vector<Eigen::Vector3d>& source_normals,
              const std::vector<Eigen::Vector3d>& target_normals)
{
    ICPResult result;
    std::vector<Eigen::Vector3d> current = source;

    // Auto-estimate target normals if needed
    std::vector<Eigen::Vector3d> tgt_normals = target_normals;

    if (settings.method == ICPMethod::PointToPlane ||
        settings.method == ICPMethod::PlaneToPlane) {
        if (tgt_normals.empty())
            tgt_normals = estimate_normals(target);
    }

    double prev_error = std::numeric_limits<double>::max();

    for (int iter = 0; iter < settings.max_iterations; ++iter) {
        auto correspondences = find_closest_points(current, target, settings.nn_method);

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
        double s = 1.0;

        switch (settings.method) {
        case ICPMethod::PointToPoint:
            compute_transform(current, target, correspondences, settings, R, t, s);
            break;
        case ICPMethod::PointToPlane:
            compute_transform_point_to_plane(
                current, target, correspondences, tgt_normals, R, t);
            break;
        case ICPMethod::PlaneToPlane: {
            // Re-estimate source normals from current points each iteration
            auto current_src_normals = estimate_normals(current);
            compute_transform_plane_to_plane(
                current, target, correspondences,
                current_src_normals, tgt_normals, R, t);
            break;
        }
        }

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
