#include "icp.h"
#include <cmath>
#include <cstdio>

static std::vector<Eigen::Vector3d> make_hemisphere(double r, int n) {
    std::vector<Eigen::Vector3d> pts;
    for (int i = 0; i <= n; ++i) {
        double phi = M_PI / 2.0 * i / n;
        int n_theta = std::max(1, static_cast<int>(n * std::sin(phi)));
        for (int j = 0; j < n_theta; ++j) {
            double theta = 2.0 * M_PI * j / n_theta;
            pts.push_back({r * std::sin(phi) * std::cos(theta),
                           r * std::sin(phi) * std::sin(theta),
                           r * std::cos(phi)});
        }
    }
    return pts;
}

static void print_result(const char* name, const ICPResult& res) {
    std::printf("=== %s ===\n", name);
    std::printf("  Iterations: %d\n", res.iterations);
    std::printf("  Error:      %.2e\n", res.error);
    std::printf("  Rotation:\n");
    for (int i = 0; i < 3; ++i)
        std::printf("    [%8.5f %8.5f %8.5f]\n",
                    res.rotation(i, 0), res.rotation(i, 1), res.rotation(i, 2));
    std::printf("  Translation: [%.5f, %.5f, %.5f]\n\n",
                res.translation(0), res.translation(1), res.translation(2));
}

int main() {
    auto target = make_hemisphere(2.0, 10);
    std::printf("Generated %zu points on a hemisphere\n\n", target.size());

    // Apply a known transform: 15-degree rotation around Z + translation
    double angle = M_PI / 12.0;
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d t(0.3, 0.2, 0.1);

    std::vector<Eigen::Vector3d> source(target.size());
    for (size_t i = 0; i < target.size(); ++i)
        source[i] = R.transpose() * (target[i] - t);

    std::printf("Ground truth:\n");
    std::printf("  Rotation:    %.1f deg around Z\n", angle * 180.0 / M_PI);
    std::printf("  Translation: [%.3f, %.3f, %.3f]\n\n", t(0), t(1), t(2));

    ICPSettings settings;
    settings.max_iterations = 100;

    settings.method = ICPMethod::PointToPoint;
    print_result("Point-to-Point", icp(source, target, settings));

    settings.method = ICPMethod::PointToPlane;
    print_result("Point-to-Plane", icp(source, target, settings));

    settings.method = ICPMethod::PlaneToPlane;
    print_result("Plane-to-Plane", icp(source, target, settings));

    return 0;
}
