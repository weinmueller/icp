#include "icp.h"
#include "pointcloud_io.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>

#define ASSERT_NEAR(a, b, eps)                                              \
    do {                                                                    \
        if (std::abs((a) - (b)) > (eps)) {                                  \
            std::printf("FAIL: %s:%d: |%f - %f| > %e\n",                   \
                        __FILE__, __LINE__, (double)(a), (double)(b),       \
                        (double)(eps));                                     \
            return 1;                                                       \
        }                                                                   \
    } while (0)

static std::string data_dir;

int test_file_translation() {
    auto source = load_pointcloud(data_dir + "/source.xyz");
    auto target = load_pointcloud(data_dir + "/target_translated.xyz");

    auto res = icp(source, target);

    // Expected translation: (1, 0.5, -0.3)
    ASSERT_NEAR(res.translation(0), 1.0,  1e-4);
    ASSERT_NEAR(res.translation(1), 0.5,  1e-4);
    ASSERT_NEAR(res.translation(2), -0.3, 1e-4);
    ASSERT_NEAR(res.error, 0.0, 1e-8);
    return 0;
}

struct TestEntry {
    const char* name;
    std::function<int()> fn;
};

static const TestEntry tests[] = {
    {"file_translation", test_file_translation},
};

int main(int argc, char* argv[]) {
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
