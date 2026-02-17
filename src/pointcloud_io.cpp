#include "pointcloud_io.h"
#include <fstream>
#include <sstream>
#include <stdexcept>

std::vector<Eigen::Vector3d> load_pointcloud(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open())
        throw std::runtime_error("Cannot open file: " + path);

    std::vector<Eigen::Vector3d> points;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream ss(line);
        double x, y, z;
        if (ss >> x >> y >> z)
            points.push_back({x, y, z});
    }
    return points;
}

void save_pointcloud(const std::string& path,
                     const std::vector<Eigen::Vector3d>& points) {
    std::ofstream out(path);
    if (!out.is_open())
        throw std::runtime_error("Cannot open file: " + path);

    out << "# x y z\n";
    out << std::fixed;
    for (const auto& p : points)
        out << p.x() << " " << p.y() << " " << p.z() << "\n";
}
