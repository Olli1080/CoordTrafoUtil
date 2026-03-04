#include <base-transformation/TransformationHelper.h>
#include <iostream>
#include <vector>

using namespace Transformation;

// 1. A simple custom 3D point structure
struct MyPoint3D {
    double data[3];
};

// 2. Specialize VectorTraits for MyPoint3D
// This "teaches" CoordTrafoUtil how to handle MyPoint3D
template<>
struct Transformation::VectorTraits<MyPoint3D, double> {
    using type = MyPoint3D;
    static double get_x(const MyPoint3D& v) { return v.data[0]; }
    static double get_y(const MyPoint3D& v) { return v.data[1]; }
    static double get_z(const MyPoint3D& v) { return v.data[2]; }
    static double get_idx(const MyPoint3D& v, size_t i) { return v.data[i]; }

    static void set_x(MyPoint3D& v, double val) { v.data[0] = val; }
    static void set_y(MyPoint3D& v, double val) { v.data[1] = val; }
    static void set_z(MyPoint3D& v, double val) { v.data[2] = val; }
    static void set_idx(MyPoint3D& v, size_t i, double val) { v.data[i] = val; }
};

int main() {
    auto source = TransformationMetaBuilder().build(); // Default X+, Y+, Z+
    auto target = TransformationMetaBuilder()
        .right(Axis::Y, AxisDirection::POSITIVE) // Swap X and Y
        .forward(Axis::X, AxisDirection::POSITIVE)
        .build();

    TransformationConverter<double> converter(source, target);

    MyPoint3D p1 = {1.0, 2.0, 3.0};
    MyPoint3D p2;

    // Use our custom type directly with the library!
    converter.convert_point(p1, p2);

    std::cout << "Original: " << p1.data[0] << ", " << p1.data[1] << ", " << p1.data[2] << std::endl;
    std::cout << "Swapped:  " << p2.data[0] << ", " << p2.data[1] << ", " << p2.data[2] << std::endl;

    return 0;
}
