#include "icp.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <vector>

#define ASSERT_NEAR(a, b, eps)                                              \
    do {                                                                    \
        if (std::abs((a) - (b)) > (eps)) {                                  \
            std::printf("FAIL: %s:%d: |%f - %f| > %e\n",                   \
                        __FILE__, __LINE__, (double)(a), (double)(b),       \
                        (double)(eps));                                     \
            return 1;                                                       \
        }                                                                   \
    } while (0)

static std::vector<Eigen::Vector3d> make_points() {
    return {
        {0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 4.0},
        {1.0, 1.0, 0.0}, {2.0, 0.5, 1.0}, {0.5, 1.5, 2.0}, {1.5, 2.5, 3.0}
    };
}

static std::vector<Eigen::Vector3d> apply_transform(
    const std::vector<Eigen::Vector3d>& pts,
    const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
    std::vector<Eigen::Vector3d> out(pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
        out[i] = R * pts[i] + t;
    return out;
}

int test_identity() {
    auto pts = make_points();
    auto res = icp(pts, pts);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            ASSERT_NEAR(res.rotation(i, j), (i == j) ? 1.0 : 0.0, 1e-6);

    for (int i = 0; i < 3; ++i)
        ASSERT_NEAR(res.translation(i), 0.0, 1e-6);

    ASSERT_NEAR(res.error, 0.0, 1e-10);
    return 0;
}

int test_pure_translation() {
    auto target = make_points();
    Eigen::Vector3d t(2.0, -1.0, 0.5);
    auto source = apply_transform(target, Eigen::Matrix3d::Identity(), -t);

    auto res = icp(source, target);

    for (int i = 0; i < 3; ++i)
        ASSERT_NEAR(res.translation(i), t(i), 1e-4);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            ASSERT_NEAR(res.rotation(i, j), (i == j) ? 1.0 : 0.0, 1e-4);
    return 0;
}

int test_pure_rotation() {
    auto target = make_points();
    double angle = M_PI / 6;
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());

    auto source = apply_transform(target, R.transpose(), Eigen::Vector3d::Zero());
    auto res = icp(source, target);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            ASSERT_NEAR(res.rotation(i, j), R(i, j), 1e-4);
    return 0;
}

int test_rotation_and_translation() {
    auto target = make_points();
    double angle = M_PI / 8;
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());
    Eigen::Vector3d t(0.3, 0.2, 0.1);

    auto source = apply_transform(target, R.transpose(), -R.transpose() * t);
    auto res = icp(source, target);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            ASSERT_NEAR(res.rotation(i, j), R(i, j), 1e-4);

    for (int i = 0; i < 3; ++i)
        ASSERT_NEAR(res.translation(i), t(i), 1e-4);
    return 0;
}

int test_convergence_error() {
    auto target = make_points();
    Eigen::Vector3d t(0.5, 0.0, 0.0);
    auto source = apply_transform(target, Eigen::Matrix3d::Identity(), -t);

    auto res = icp(source, target);
    ASSERT_NEAR(res.error, 0.0, 1e-8);
    return 0;
}

int test_translation_only() {
    auto target = make_points();
    Eigen::Vector3d t(1.0, -0.5, 0.3);
    auto source = apply_transform(target, Eigen::Matrix3d::Identity(), -t);

    ICPSettings settings;
    settings.rotation = false;
    settings.translation = true;
    auto res = icp(source, target, settings);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            ASSERT_NEAR(res.rotation(i, j), (i == j) ? 1.0 : 0.0, 1e-10);

    for (int i = 0; i < 3; ++i)
        ASSERT_NEAR(res.translation(i), t(i), 1e-4);
    return 0;
}

int test_with_scaling() {
    auto target = make_points();
    double s = 1.2;
    std::vector<Eigen::Vector3d> source(target.size());
    for (size_t i = 0; i < target.size(); ++i)
        source[i] = target[i] / s;

    ICPSettings settings;
    settings.scaling = true;
    auto res = icp(source, target, settings);

    ASSERT_NEAR(res.scale, s, 1e-4);
    ASSERT_NEAR(res.error, 0.0, 1e-6);
    return 0;
}

int test_no_rotation_no_translation() {
    auto pts = make_points();
    Eigen::Vector3d t(0.5, 0.5, 0.5);
    auto source = apply_transform(pts, Eigen::Matrix3d::Identity(), -t);

    ICPSettings settings;
    settings.rotation = false;
    settings.translation = false;
    auto res = icp(source, pts, settings);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            ASSERT_NEAR(res.rotation(i, j), (i == j) ? 1.0 : 0.0, 1e-10);

    for (int i = 0; i < 3; ++i)
        ASSERT_NEAR(res.translation(i), 0.0, 1e-10);
    return 0;
}

struct TestEntry {
    const char* name;
    std::function<int()> fn;
};

static const TestEntry tests[] = {
    {"identity",                   test_identity},
    {"pure_translation",           test_pure_translation},
    {"pure_rotation",              test_pure_rotation},
    {"rotation_and_translation",   test_rotation_and_translation},
    {"convergence_error",          test_convergence_error},
    {"translation_only",           test_translation_only},
    {"with_scaling",               test_with_scaling},
    {"no_rotation_no_translation", test_no_rotation_no_translation},
};

int main(int argc, char* argv[]) {
    if (argc == 2 && std::strcmp(argv[1], "--list") == 0) {
        for (const auto& t : tests)
            std::printf("%s\n", t.name);
        return 0;
    }

    if (argc == 2) {
        for (const auto& t : tests) {
            if (std::strcmp(argv[1], t.name) == 0) {
                int rc = t.fn();
                std::printf("%s: %s\n", t.name, rc == 0 ? "PASS" : "FAIL");
                return rc;
            }
        }
        std::fprintf(stderr, "Unknown test: %s\n", argv[1]);
        return 1;
    }

    int passed = 0, failed = 0;
    for (const auto& t : tests) {
        int rc = t.fn();
        std::printf("%-45s %s\n", t.name, rc == 0 ? "PASS" : "FAIL");
        rc == 0 ? passed++ : failed++;
    }
    std::printf("\n%d passed, %d failed\n", passed, failed);
    return failed > 0 ? 1 : 0;
}
