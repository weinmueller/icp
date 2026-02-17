# ICP — Iterative Closest Point Registration

Minimal C++ implementation of the ICP algorithm for 3D rigid point-set registration using Eigen.

## Features

- SVD-based rigid alignment (rotation + translation)
- Optional uniform scaling
- Configurable transform components via `ICPSettings`
- Simple `.xyz` point cloud file I/O

## Dependencies

- C++17
- [Eigen3](https://eigen.tuxfamily.org)
- CMake ≥ 3.10

## Build

```bash
cmake -B build
cmake --build build
```

## Usage

```cpp
#include "icp.h"

std::vector<Eigen::Vector3d> source = { /* ... */ };
std::vector<Eigen::Vector3d> target = { /* ... */ };

// Default: rotation + translation
ICPResult result = icp(source, target);

// Custom settings
ICPSettings settings;
settings.scaling = true;       // enable uniform scaling
settings.rotation = false;     // disable rotation
settings.max_iterations = 100;
settings.tolerance = 1e-8;

ICPResult result = icp(source, target, settings);

// Access results
result.rotation;    // Eigen::Matrix3d
result.translation; // Eigen::Vector3d
result.scale;       // double
result.iterations;  // int
result.error;       // double (mean squared error)
```

### Point cloud files

Load and save point clouds as plain text (one `x y z` per line, `#` for comments):

```cpp
#include "pointcloud_io.h"

auto points = load_pointcloud("cloud.xyz");
save_pointcloud("output.xyz", points);
```

File format:

```
# my point cloud
0.0 0.0 0.0
3.0 0.0 0.0
0.0 2.0 0.0
```

## Tests

```bash
ctest --test-dir build -j$(nproc)                # all tests in parallel
ctest --test-dir build -L rigid                   # core algorithm tests
ctest --test-dir build -L settings                # settings/mode tests
ctest --test-dir build -L fileio                  # point cloud I/O tests
ctest --test-dir build -L integration             # end-to-end file tests
ctest --test-dir build --output-on-failure        # show output on failure
```

To add your own test data, place `.xyz` files in `tests/data/` and register new test cases in `tests/CMakeLists.txt` via `icp_add_test()`.

## Project Structure

```
├── CMakeLists.txt
├── src/
│   ├── CMakeLists.txt
│   ├── icp.h / icp.cpp
│   └── pointcloud_io.h / pointcloud_io.cpp
└── tests/
    ├── CMakeLists.txt
    ├── test_icp.cpp
    ├── test_pointcloud_io.cpp
    ├── test_file_icp.cpp
    └── data/
        ├── source.xyz
        └── target_translated.xyz
```
