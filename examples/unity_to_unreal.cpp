#include <base-transformation/TransformationHelper.h>
#include <base-transformation/Plugins/eigen.h>
#include <Eigen/Dense>
#include <iostream>

using namespace Transformation;

int main() {
    // 1. Define Unity Coordinate System:
    // Left-handed, X: Right, Y: Up, Z: Forward. Meters (1/1).
    auto unity = TransformationMetaBuilder()
        .right(Axis::X, AxisDirection::POSITIVE)
        .forward(Axis::Z, AxisDirection::POSITIVE)
        .up(Axis::Y, AxisDirection::POSITIVE)
        .scale(1, 1)
        .build();

    // 2. Define Unreal Coordinate System:
    // Left-handed, X: Forward, Y: Right, Z: Up. Centimeters (1/100).
    auto unreal = TransformationMetaBuilder()
        .right(Axis::Y, AxisDirection::POSITIVE)
        .forward(Axis::X, AxisDirection::POSITIVE)
        .up(Axis::Z, AxisDirection::POSITIVE)
        .scale(1, 100)
        .build();

    // 3. Create the converter
    TransformationConverter converter(unity, unreal);

    // 4. Convert a point
    Eigen::Vector3f point_unity(1.0f, 2.0f, 3.0f); // 1m Right, 2m Up, 3m Forward
    Eigen::Vector3f point_unreal;
    
    converter.convert_point(point_unity, point_unreal);

    std::cout << "Unity Point:  " << point_unity.transpose() << " (meters)" << std::endl;
    std::cout << "Unreal Point: " << point_unreal.transpose() << " (cm)" << std::endl;
    // Expected: X: 300, Y: 100, Z: 200

    // 5. Convert a rotation
    Eigen::Quaternionf rot_unity = Eigen::Quaternionf(Eigen::AngleAxisf(0.5f, Eigen::Vector3f::UnitY()));
    Eigen::Quaternionf rot_unreal;

    converter.convert_quaternion(rot_unity, rot_unreal);
    std::cout << "Unreal Rotation (W): " << rot_unreal.w() << std::endl;

    return 0;
}
