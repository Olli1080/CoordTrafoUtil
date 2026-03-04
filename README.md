# CoordTrafoUtil

A lightweight, header-only C++20 utility library for transforming coordinates, vectors, quaternions, and matrices between different 3D coordinate systems.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Features

- **Generic & Extensible**: Support for any math library (Eigen, PCL, etc.) via a simple `Traits` system.
- **Modern C++**: Built with C++20 concepts and `constexpr` support.
- **Zero Overhead**: Header-only architecture with highly optimized mapping logic.
- **Robust**: Handles axis swaps, inversions (handedness changes), and scale/unit conversions (e.g., meters to centimeters).
- **Type Safe**: Prevents invalid coordinate system definitions at compile-time or runtime.

## Quick Start

### 1. Define Coordinate Systems
Use the `TransformationMetaBuilder` to define your source and target systems.

```cpp
#include <base-transformation/TransformationHelper.h>

using namespace Transformation;

// Standard Right-Handed (X: Right, Y: Forward, Z: Up) in meters
auto source = TransformationMetaBuilder()
    .right(Axis::X, AxisDirection::POSITIVE)
    .forward(Axis::Y, AxisDirection::POSITIVE)
    .up(Axis::Z, AxisDirection::POSITIVE)
    .scale(1, 1) // 1/1 = meters
    .build();

// Unity-style Left-Handed (X: Right, Y: Up, Z: Forward) in centimeters
auto target = TransformationMetaBuilder()
    .right(Axis::X, AxisDirection::POSITIVE)
    .forward(Axis::Z, AxisDirection::POSITIVE)
    .up(Axis::Y, AxisDirection::POSITIVE)
    .scale(1, 100) // 1/100 = cm
    .build();
```

### 2. Convert Data
Create a `TransformationConverter` and pass your math types directly.

```cpp
#include <base-transformation/Plugins/eigen.h>
#include <Eigen/Dense>

TransformationConverter converter(source, target);

Eigen::Vector3f point_m(1.0f, 2.0f, 3.0f);
Eigen::Vector3f point_cm;

// Automatic axis mapping and scaling
converter.convert_point(point_m, point_cm); 
```

## Adding Support for Custom Types
To support your own vector or matrix types, simply specialize the corresponding `Traits` template:

```cpp
template<>
struct Transformation::VectorTraits<MyVec3, float> {
    static float get_x(const MyVec3& v) { return v.x; }
    // ... implement set_x, get_idx, etc.
};
```

## Installation

### Via vcpkg
```bash
vcpkg install base-transformation
```

### Via CMake
```cmake
find_package(base-transformation CONFIG REQUIRED)
target_link_libraries(main PRIVATE Transformation::Core)

# Optional: Enable Eigen/PCL plugins
find_package(base-transformation CONFIG REQUIRED COMPONENTS plugins)
target_link_libraries(main PRIVATE Transformation::Plugins)
```

## Documentation
API documentation is available via Doxygen. Run `doxygen Doxyfile` in the root directory to generate HTML docs.

## License
This project is licensed under the [MIT License](LICENSE).
