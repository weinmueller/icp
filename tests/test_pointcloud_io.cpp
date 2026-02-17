#include "pointcloud_io.h"
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

#define ASSERT_EQ(a, b)                                                     \
    do {                                                                    \
        if ((a) != (b)) {                                                   \
            std::printf("FAIL: %s:%d: %d != %d\n",                         \
                        __FILE__, __LINE__, (int)(a), (int)(b));            \
            return 1;                                                       \
        }                                                                   \
    } while (0)

static std::string data_dir;

int test_load() {
    auto pts = load_pointcloud(data_dir + "/source.xyz");
    ASSERT_EQ(pts.size(), 8);
    ASSERT_NEAR(pts[0].x(), 0.0, 1e-10);
    ASSERT_NEAR(pts[1].x(), 3.0, 1e-10);
    ASSERT_NEAR(pts[3].z(), 4.0, 1e-10);
    return 0;
}

int test_load_translated() {
    auto pts = load_pointcloud(data_dir + "/target_translated.xyz");
    ASSERT_EQ(pts.size(), 8);
    // First point should be (1, 0.5, -0.3)
    ASSERT_NEAR(pts[0].x(), 1.0, 1e-10);
    ASSERT_NEAR(pts[0].y(), 0.5, 1e-10);
    ASSERT_NEAR(pts[0].z(), -0.3, 1e-10);
    return 0;
}

int test_roundtrip() {
    std::vector<Eigen::Vector3d> original = {
        {1.5, 2.5, 3.5}, {-1.0, 0.0, 1.0}
    };
    std::string tmp = data_dir + "/roundtrip_tmp.xyz";
    save_pointcloud(tmp, original);
    auto loaded = load_pointcloud(tmp);

    ASSERT_EQ(loaded.size(), 2);
    for (size_t i = 0; i < original.size(); ++i) {
        ASSERT_NEAR(loaded[i].x(), original[i].x(), 1e-6);
        ASSERT_NEAR(loaded[i].y(), original[i].y(), 1e-6);
        ASSERT_NEAR(loaded[i].z(), original[i].z(), 1e-6);
    }
    std::remove(tmp.c_str());
    return 0;
}

int test_load_missing_file() {
    try {
        load_pointcloud(data_dir + "/nonexistent.xyz");
        std::printf("FAIL: expected exception\n");
        return 1;
    } catch (const std::runtime_error&) {
        return 0;
    }
}

struct TestEntry {
    const char* name;
    std::function<int()> fn;
};

static const TestEntry tests[] = {
    {"load",              test_load},
    {"load_translated",   test_load_translated},
    {"roundtrip",         test_roundtrip},
    {"load_missing_file", test_load_missing_file},
};

int main(int argc, char* argv[]) {
    // DATA_DIR is passed via CMake define or first non-test argument
    const char* env = std::getenv("ICP_TEST_DATA_DIR");
    if (env)
        data_dir = env;

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
