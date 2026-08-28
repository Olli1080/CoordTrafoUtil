#include <base-transformation/TransformationHelper.h>
#include <iostream>
#include <vector>

using namespace Transformation;

// 1. A simple custom 3D point structure
struct MyPoint3D {
    double data[3];
};

// 2. Signal that traits are enabled for this type
template<> struct Transformation::TraitsEnabled<MyPoint3D> : std::true_type {};

// 3. Specialize VectorTraits for MyPoint3D.
// Inheriting from VectorAccessByIndex synthesises get_x/y/z and set_x/y/z from
// the get_idx / set_idx pair, so only three members need to be written.
template<>
struct Transformation::VectorTraits<MyPoint3D, double>
    : Transformation::VectorAccessByIndex<Transformation::VectorTraits<MyPoint3D, double>, double> {
    using type = MyPoint3D;
    static double get_idx(const MyPoint3D& v, size_t i) { return v.data[i]; }
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
