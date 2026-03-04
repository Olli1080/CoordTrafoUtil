# CoordTrafoUtil

A lightweight, header-only C++20 utility library for transforming coordinates, vectors, quaternions, and matrices between different 3D coordinate systems.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Features

- **Generic & Extensible**: Support for any math library (Eigen, PCL, etc.) via a non-intrusive `Traits` system.
- **Modern C++**: Built with C++20 concepts and full `constexpr` support.
- **Zero Overhead**: Header-only architecture with optimized sparse mapping logic.
- **Precision Agnostic**: Support for `float`, `double`, or arbitrary scalar types.
- **Robust**: Handles axis swaps, handedness changes, and scale/unit conversions.
- **Safety**: Prevents invalid coordinate systems and provides clear compile-time error messages.

## Quick Start

### 1. Define Coordinate Systems
Use the `Presets` for common systems or the `TransformationMetaBuilder` for custom ones.

```cpp
#include <base-transformation/TransformationHelper.h>

using namespace Transformation;

// Using Presets
auto source = Presets::Unity(); // Left-handed, X:Right, Y:Up, Z:Forward

// Using the Builder with string literals
auto target = TransformationMetaBuilder()
    .right("Y+"_a)   // Alignment literals: "X+", "X-", "Y+", etc.
    .forward("X+"_a)
    .up("Z+"_a)
    .scale(1, 100)   // cm to meters
    .build();
```

### 2. Convert Data
Create a `TransformationConverter` and pass your math types directly.

```cpp
#include <base-transformation/Plugins/eigen.h>
#include <Eigen/Dense>

// Specify precision (defaults to float)
TransformationConverter<double> converter(source, target);

Eigen::Vector3d point_unity(1.0, 2.0, 3.0);
Eigen::Vector3d point_target;

// Reference-based conversion
converter.convert_point(point_unity, point_target); 

// OR Return-by-value
auto p2 = converter.convert_point<Eigen::Vector3d>(point_unity);
```

## Batch Processing
For high-performance tasks, use the batch conversion methods which are optimized for compiler auto-vectorization.

```cpp
std::vector<Eigen::Vector3f> buffer_in = { ... };
std::vector<Eigen::Vector3f> buffer_out(buffer_in.size());

converter.convert_points(std::span(buffer_in), std::span(buffer_out));
```

## Adding Support for Custom Types
Specialize `TraitsEnabled` and the corresponding `Traits` struct:

```cpp
struct MyVec { double x, y, z; };

template<> struct Transformation::TraitsEnabled<MyVec> : std::true_type {};

template<>
struct Transformation::VectorTraits<MyVec, double> {
    using type = MyVec;
    static double get_x(const MyVec& v) { return v.x; }
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
API documentation is automatically deployed to GitHub Pages on every push. You can also generate it locally by running `doxygen Doxyfile`.

## License
This project is licensed under the [MIT License](LICENSE).
