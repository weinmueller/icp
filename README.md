# ICP — Iterative Closest Point Registration

Minimal C++ implementation of the ICP algorithm for 3D rigid point-set registration using Eigen.

## Features

- Multiple ICP methods: point-to-point (SVD), point-to-plane, and plane-to-plane (symmetric)
- Automatic surface normal estimation via PCA on k-nearest neighbors
- SVD-based rigid alignment (rotation + translation)
- Optional uniform scaling
- Configurable transform components via `ICPSettings`
- Nearest-neighbor search: brute-force O(n*m) or KD-tree O(n log m)
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

// Default: rotation + translation, KD-tree nearest neighbor
ICPResult result = icp(source, target);

// Custom settings
ICPSettings settings;
settings.method = ICPMethod::PointToPlane;    // or PointToPoint (default), PlaneToPlane
settings.rotation = false;                    // disable rotation
settings.translation = true;                  // enable translation (default)
settings.scaling = true;                      // enable uniform scaling
settings.nn_method = NNMethod::BruteForce;    // or NNMethod::KDTree (default)
settings.max_iterations = 100;
settings.tolerance = 1e-8;

ICPResult result = icp(source, target, settings);

// With pre-computed normals (optional — auto-estimated if omitted)
auto src_normals = estimate_normals(source);
auto tgt_normals = estimate_normals(target);
ICPResult result = icp(source, target, settings, src_normals, tgt_normals);

// Access results
result.rotation;    // Eigen::Matrix3d
result.translation; // Eigen::Vector3d
result.scale;       // double
result.iterations;  // int
result.error;       // double (mean squared error)
```

### Settings

| Setting | Default | Description |
|---|---|---|
| `method` | `PointToPoint` | ICP method (`PointToPoint`, `PointToPlane`, or `PlaneToPlane`) |
| `rotation` | `true` | Enable rotation recovery |
| `translation` | `true` | Enable translation recovery |
| `scaling` | `false` | Enable uniform scale recovery |
| `nn_method` | `KDTree` | Nearest-neighbor method (`KDTree` or `BruteForce`) |
| `max_iterations` | `50` | Maximum ICP iterations |
| `tolerance` | `1e-6` | Convergence threshold on mean squared error change |

### ICP Methods

| Method | Description |
|---|---|
| `PointToPoint` | Classic SVD-based (Besl & McKay). Minimizes point-to-point distances. |
| `PointToPlane` | Linearized least-squares. Minimizes distances projected onto target normals. Converges faster on smooth surfaces. |
| `PlaneToPlane` | Symmetric variant using normals from both clouds. More robust when both clouds have noise. |

Point-to-plane and plane-to-plane require surface normals. If not provided, they are automatically estimated via PCA on k-nearest neighbors.

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
ctest --test-dir build -L method                  # ICP method tests
ctest --test-dir build -L kdtree                  # KD-tree tests
ctest --test-dir build -L fileio                  # point cloud I/O tests
ctest --test-dir build -L integration             # end-to-end file tests
ctest --test-dir build --output-on-failure        # show output on failure
```

| Label | Tests | Description |
|---|---|---|
| `rigid` | 5 | Core ICP: identity, translation, rotation, combined, convergence |
| `settings` | 3 | Mode toggles: translation-only, scaling, disabled |
| `method` | 3 | ICP methods: point-to-plane, plane-to-plane, method comparison |
| `kdtree` | 3 | KD-tree: translation, rotation, brute-force comparison |
| `fileio` | 4 | Point cloud load/save/roundtrip/error handling |
| `integration` | 1 | End-to-end: load files, run ICP, verify |

To add your own test data, place `.xyz` files in `tests/data/` and register new test cases in `tests/CMakeLists.txt` via `icp_add_test()`.

## Iteration Callback

You can attach a callback to observe each ICP iteration (useful for debugging or visualization):

```cpp
ICPSettings settings;
settings.iteration_callback = [](const ICPIterationData& data) {
    std::printf("iter %d: error=%.4e\n", data.iteration, data.error);
    // data.source_points — current transformed positions
    // data.correspondences — nearest-neighbor indices into target
};
icp(source, target, settings);
```

## Viewer

An interactive OpenGL viewer is built automatically if GLFW is installed:

```bash
# Install GLFW (Ubuntu/Debian)
sudo apt install libglfw3-dev

cmake -B build && cmake --build build
./build/examples/viewer         # standalone viewer
./build/examples/demo --viewer  # run demo then open viewer
```

The `--viewer` flag runs all three ICP methods, collects iteration snapshots, and opens the viewer window. If GLFW was not found at build time, `--viewer` prints an error. Without the flag, `demo` behaves as a normal CLI program.

Controls:

| Key | Action |
|---|---|
| Left / Right | Step through iterations |
| Space | Toggle auto-play |
| Home / End | Jump to first / last iteration |
| Mouse drag | Orbit camera |
| Scroll | Zoom |
| Q / Escape | Quit |

## Project Structure

```
├── CMakeLists.txt
├── src/
│   ├── CMakeLists.txt
│   ├── icp.h / icp.cpp          # ICP algorithm
│   ├── kdtree.h                  # Minimal 3D KD-tree (nearest + k-nearest)
│   ├── normals.h                 # Surface normal estimation via PCA
│   └── pointcloud_io.h / .cpp   # Point cloud file I/O
├── examples/
│   ├── CMakeLists.txt
│   ├── demo.cpp                  # CLI demo (all three ICP methods)
│   └── viewer.cpp                # Interactive OpenGL viewer (requires GLFW)
└── tests/
    ├── CMakeLists.txt
    ├── test_icp.cpp              # Algorithm + KD-tree tests
    ├── test_pointcloud_io.cpp    # File I/O tests
    ├── test_file_icp.cpp         # Integration tests
    └── data/
        ├── source.xyz
        └── target_translated.xyz
```
