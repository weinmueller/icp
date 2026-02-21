#include "icp.h"
#include <cmath>
#include <cstdio>
#include <cstring>

#if __has_include(<GLFW/glfw3.h>)
#define ICP_HAS_VIEWER 1
#include <GLFW/glfw3.h>
#else
#define ICP_HAS_VIEWER 0
#endif

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

static void make_test_clouds(std::vector<Eigen::Vector3d>& source,
                             std::vector<Eigen::Vector3d>& target) {
    target = make_hemisphere(2.0, 10);

    double angle = M_PI / 12.0;
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d t(0.3, 0.2, 0.1);

    source.resize(target.size());
    for (size_t i = 0; i < target.size(); ++i)
        source[i] = R.transpose() * (target[i] - t);
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

#if ICP_HAS_VIEWER

// ── Viewer state ────────────────────────────────────────────────────────────

struct Snapshot {
    const char* method;
    double error;
    std::vector<Eigen::Vector3d> source_points;
    std::vector<int> correspondences;
};

static std::vector<Snapshot> g_snapshots;
static std::vector<Eigen::Vector3d> g_target;

static float g_rot_x = 30.0f;
static float g_rot_y = -30.0f;
static float g_zoom = 6.0f;
static bool g_dragging = false;
static double g_last_mx, g_last_my;

static int g_current = 0;
static int g_last_title_update = -1;
static bool g_last_title_playing = false;
static bool g_playing = false;
static double g_last_step_time = 0.0;
static const double STEP_INTERVAL = 0.3;

// ── GLFW callbacks ──────────────────────────────────────────────────────────

static void key_callback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
    if (action != GLFW_PRESS && action != GLFW_REPEAT) return;
    int max_idx = static_cast<int>(g_snapshots.size()) - 1;
    switch (key) {
        case GLFW_KEY_RIGHT:
            g_playing = false;
            if (g_current < max_idx) ++g_current;
            break;
        case GLFW_KEY_LEFT:
            g_playing = false;
            if (g_current > 0) --g_current;
            break;
        case GLFW_KEY_SPACE:
            g_playing = !g_playing;
            g_last_step_time = glfwGetTime();
            break;
        case GLFW_KEY_HOME:
            g_playing = false;
            g_current = 0;
            break;
        case GLFW_KEY_END:
            g_playing = false;
            g_current = max_idx;
            break;
        case GLFW_KEY_ESCAPE:
        case GLFW_KEY_Q:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
    }
}

static void mouse_button_callback(GLFWwindow* window, int button, int action, int /*mods*/) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        g_dragging = (action == GLFW_PRESS);
        if (g_dragging)
            glfwGetCursorPos(window, &g_last_mx, &g_last_my);
    }
}

static void cursor_pos_callback(GLFWwindow* /*window*/, double mx, double my) {
    if (g_dragging) {
        g_rot_y += static_cast<float>(mx - g_last_mx) * 0.5f;
        g_rot_x += static_cast<float>(my - g_last_my) * 0.5f;
        g_rot_x = std::max(-90.0f, std::min(90.0f, g_rot_x));
        g_last_mx = mx;
        g_last_my = my;
    }
}

static void scroll_callback(GLFWwindow* /*window*/, double /*xoff*/, double yoff) {
    g_zoom -= static_cast<float>(yoff) * 0.5f;
    g_zoom = std::max(1.0f, std::min(30.0f, g_zoom));
}

// ── Rendering ───────────────────────────────────────────────────────────────

static void render(GLFWwindow* window) {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double aspect = static_cast<double>(width) / std::max(height, 1);
    double fov = 45.0;
    double near_plane = 0.1, far_plane = 100.0;
    double top = near_plane * std::tan(fov * M_PI / 360.0);
    double right = top * aspect;
    glFrustum(-right, right, -top, top, near_plane, far_plane);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -g_zoom);
    glRotatef(g_rot_x, 1.0f, 0.0f, 0.0f);
    glRotatef(g_rot_y, 0.0f, 1.0f, 0.0f);

    glEnable(GL_DEPTH_TEST);
    glPointSize(4.0f);

    const auto& snap = g_snapshots[static_cast<size_t>(g_current)];

    // Target points (blue)
    glColor3f(0.3f, 0.5f, 1.0f);
    glBegin(GL_POINTS);
    for (const auto& p : g_target)
        glVertex3d(p.x(), p.y(), p.z());
    glEnd();

    // Source points (red)
    glColor3f(1.0f, 0.3f, 0.3f);
    glBegin(GL_POINTS);
    for (const auto& p : snap.source_points)
        glVertex3d(p.x(), p.y(), p.z());
    glEnd();

    // Correspondence lines (green, thin)
    glColor4f(0.2f, 0.8f, 0.2f, 0.4f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (size_t i = 0; i < snap.source_points.size(); ++i) {
        const auto& s = snap.source_points[i];
        const auto& t = g_target[static_cast<size_t>(snap.correspondences[i])];
        glVertex3d(s.x(), s.y(), s.z());
        glVertex3d(t.x(), t.y(), t.z());
    }
    glEnd();
}

static void update_title(GLFWwindow* window) {
    if (g_current == g_last_title_update && g_playing == g_last_title_playing)
        return;
    g_last_title_update = g_current;
    g_last_title_playing = g_playing;
    const auto& snap = g_snapshots[static_cast<size_t>(g_current)];

    // Count iterations for the current method and find position within it
    int method_idx = 0, method_total = 0;
    for (size_t i = 0; i < g_snapshots.size(); ++i) {
        if (g_snapshots[i].method == snap.method) {
            if (static_cast<int>(i) <= g_current)
                method_idx = method_total;
            ++method_total;
        }
    }

    char title[256];
    std::snprintf(title, sizeof(title),
                  "ICP Viewer — %s — iteration %d/%d — error %.4e%s",
                  snap.method,
                  method_idx + 1,
                  method_total,
                  snap.error,
                  g_playing ? " [playing]" : "");
    glfwSetWindowTitle(window, title);
}

static int run_viewer(const std::vector<Eigen::Vector3d>& source,
                      const std::vector<Eigen::Vector3d>& target) {
    g_target = target;

    // Run all three ICP methods, collecting snapshots
    const ICPMethod methods[] = {ICPMethod::PointToPoint, ICPMethod::PointToPlane, ICPMethod::PlaneToPlane};
    const char* names[] = {"Point-to-Point", "Point-to-Plane", "Plane-to-Plane"};

    ICPSettings settings;
    settings.max_iterations = 100;
    for (int m = 0; m < 3; ++m) {
        settings.method = methods[m];
        settings.iteration_callback = [name = names[m]](const ICPIterationData& data) {
            g_snapshots.push_back({name, data.error,
                                   data.source_points, data.correspondences});
        };
        std::printf("Running %s...\n", names[m]);
        ICPResult result = icp(source, target, settings);
        std::printf("  %d iterations, error %.4e\n", result.iterations, result.error);
    }

    std::printf("Captured %zu snapshots\n", g_snapshots.size());

    if (g_snapshots.empty()) {
        std::fprintf(stderr, "No iterations captured.\n");
        return 1;
    }

    // Init GLFW
    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(1024, 768, "ICP Viewer", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "Failed to create window\n");
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_pos_callback);
    glfwSetScrollCallback(window, scroll_callback);

    std::printf("\nControls:\n");
    std::printf("  Left/Right arrows — step through iterations\n");
    std::printf("  Space             — toggle auto-play\n");
    std::printf("  Home/End          — jump to first/last iteration\n");
    std::printf("  Mouse drag        — orbit camera\n");
    std::printf("  Scroll            — zoom\n");
    std::printf("  Q / Escape        — quit\n");

    while (!glfwWindowShouldClose(window)) {
        if (g_playing) {
            double now = glfwGetTime();
            if (now - g_last_step_time >= STEP_INTERVAL) {
                g_last_step_time = now;
                if (g_current < static_cast<int>(g_snapshots.size()) - 1)
                    ++g_current;
                else
                    g_playing = false;
            }
        }

        render(window);
        update_title(window);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

#endif // ICP_HAS_VIEWER

int main(int argc, char* argv[]) {
    bool viewer_mode = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--viewer") == 0)
            viewer_mode = true;
    }

    if (viewer_mode) {
#if ICP_HAS_VIEWER
        std::vector<Eigen::Vector3d> source, target;
        make_test_clouds(source, target);
        return run_viewer(source, target);
#else
        std::fprintf(stderr, "Error: --viewer requires GLFW, which was not found at build time.\n");
        return 1;
#endif
    }

    std::vector<Eigen::Vector3d> source, target;
    make_test_clouds(source, target);
    std::printf("Generated %zu points on a hemisphere\n\n", target.size());

    std::printf("Ground truth:\n");
    std::printf("  Rotation:    15.0 deg around Z\n");
    std::printf("  Translation: [0.300, 0.200, 0.100]\n\n");

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
