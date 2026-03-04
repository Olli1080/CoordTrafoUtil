#include <gtest/gtest.h>
#include "base-transformation/TransformationHelper.h"
#include "base-transformation/Plugins/eigen.h"
#include <Eigen/Dense>

using namespace Transformation;

TEST(TransformationTest, IdentityTransformation) {
    TransformationMeta origin(
        {Axis::X, AxisDirection::POSITIVE},
        {Axis::Y, AxisDirection::POSITIVE},
        {Axis::Z, AxisDirection::POSITIVE}
    );
    TransformationMeta target = origin;

    TransformationConverter conv(origin, target);
    
    Eigen::Vector3f point(1.0f, 2.0f, 3.0f);
    Eigen::Vector3f out;

    conv.convert_point(point, out);

    EXPECT_NEAR(out.x(), point.x(), 1e-5);
    EXPECT_NEAR(out.y(), point.y(), 1e-5);
    EXPECT_NEAR(out.z(), point.z(), 1e-5);
}

TEST(TransformationTest, ScaleTransformation) {
    // Meters to Centimeters
    TransformationMeta origin(
        {Axis::X, AxisDirection::POSITIVE},
        {Axis::Y, AxisDirection::POSITIVE},
        {Axis::Z, AxisDirection::POSITIVE},
        {1, 1} // meters (1/1)
    );
    TransformationMeta target(
        {Axis::X, AxisDirection::POSITIVE},
        {Axis::Y, AxisDirection::POSITIVE},
        {Axis::Z, AxisDirection::POSITIVE},
        {1, 100} // centimeters (1/100)
    );

    TransformationConverter conv(origin, target);
    
    Eigen::Vector3f point(1.0f, 1.0f, 1.0f);
    Eigen::Vector3f out;

    conv.convert_point(point, out);

    // 1m -> 100cm
    EXPECT_NEAR(out.x(), 100.0f, 1e-5);
    EXPECT_NEAR(out.y(), 100.0f, 1e-5);
    EXPECT_NEAR(out.z(), 100.0f, 1e-5);
}

TEST(TransformationTest, AxisSwap) {
    // Right: X+, Forward: Y+, Up: Z+  (Standard RH)
    TransformationMeta origin(
        {Axis::X, AxisDirection::POSITIVE},
        {Axis::Y, AxisDirection::POSITIVE},
        {Axis::Z, AxisDirection::POSITIVE}
    );
    // Right: Y+, Forward: X+, Up: Z+ (Swap X and Y)
    TransformationMeta target(
        {Axis::Y, AxisDirection::POSITIVE},
        {Axis::X, AxisDirection::POSITIVE},
        {Axis::Z, AxisDirection::POSITIVE}
    );

    TransformationConverter conv(origin, target);
    
    Eigen::Vector3f point(1.0f, 2.0f, 3.0f);
    Eigen::Vector3f out;

    conv.convert_point(point, out);

    // X becomes Y, Y becomes X, Z stays Z
    EXPECT_NEAR(out.x(), 2.0f, 1e-5);
    EXPECT_NEAR(out.y(), 1.0f, 1e-5);
    EXPECT_NEAR(out.z(), 3.0f, 1e-5);
}

TEST(TransformationTest, MatrixConversion) {
    TransformationMeta origin(
        {Axis::X, AxisDirection::POSITIVE},
        {Axis::Y, AxisDirection::POSITIVE},
        {Axis::Z, AxisDirection::POSITIVE}
    );
    // Right: X+, Forward: Z+, Up: Y+ (Swap Y and Z)
    TransformationMeta target(
        {Axis::X, AxisDirection::POSITIVE},
        {Axis::Z, AxisDirection::POSITIVE},
        {Axis::Y, AxisDirection::POSITIVE}
    );

    TransformationConverter conv(origin, target);

    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat(0, 3) = 1.0f; // Translation X
    mat(1, 3) = 2.0f; // Translation Y
    mat(2, 3) = 3.0f; // Translation Z

    Eigen::Matrix4f out;

    conv.convert_matrix(mat, out);

    // X stays X, Y becomes Z, Z becomes Y
    EXPECT_NEAR(out(0, 3), 1.0f, 1e-5);
    EXPECT_NEAR(out(1, 3), 3.0f, 1e-5);
    EXPECT_NEAR(out(2, 3), 2.0f, 1e-5);
}

TEST(TransformationTest, QuaternionConversion) {
    // Right: X+, Forward: Y+, Up: Z+ (Standard RH)
    TransformationMeta origin(
        {Axis::X, AxisDirection::POSITIVE},
        {Axis::Y, AxisDirection::POSITIVE},
        {Axis::Z, AxisDirection::POSITIVE}
    );
    // Right: X+, Forward: Z+, Up: Y+ (Swap Y and Z) - this changes handedness!
    TransformationMeta target(
        {Axis::X, AxisDirection::POSITIVE},
        {Axis::Z, AxisDirection::POSITIVE},
        {Axis::Y, AxisDirection::POSITIVE}
    );

    TransformationConverter conv(origin, target);

    // Rotation around X axis
    Eigen::Quaternionf quat = Eigen::Quaternionf(Eigen::AngleAxisf(0.5f, Eigen::Vector3f::UnitX()));
    
    Eigen::Quaternionf out;

    conv.convert_quaternion(quat, out);

    // Verify by transforming a point
    Eigen::Vector3f p(0, 1, 0); // Point on Y axis
    Eigen::Vector3f p_rot = quat * p;

    Eigen::Vector3f p_target;
    conv.convert_point(p, p_target); // Should be (0, 0, 1) in target system

    Eigen::Vector3f p_rot_target_actual = out * p_target;

    Eigen::Vector3f p_rot_target_expected;
    conv.convert_point(p_rot, p_rot_target_expected);

    EXPECT_NEAR(p_rot_target_actual.x(), p_rot_target_expected.x(), 1e-5);
    EXPECT_NEAR(p_rot_target_actual.y(), p_rot_target_expected.y(), 1e-5);
    EXPECT_NEAR(p_rot_target_actual.z(), p_rot_target_expected.z(), 1e-5);
}

TEST(TransformationTest, SourceCppRegressionCase1) {
    TransformationMeta source(
        {Axis::X, AxisDirection::POSITIVE},
        {Axis::Y, AxisDirection::NEGATIVE},
        {Axis::Z, AxisDirection::NEGATIVE}
    );

    TransformationMeta target(
        { Axis::X, AxisDirection::POSITIVE },
        { Axis::Z, AxisDirection::POSITIVE },
        { Axis::Y, AxisDirection::POSITIVE },
        std::centi{}
    );

    Eigen::Matrix4f tempMat;
    tempMat << 0, 1, 2, 3,
        10, 11, 12, 13,
        20, 21, 22, 23,
        0, 0, 0, 1;

    TransformationConverter conv(source, target);
    
    Eigen::Matrix4f out;
    conv.convert_matrix(tempMat, out);

    EXPECT_NEAR(out(0, 3), 3.0f * 100.0f, 1e-5);
    EXPECT_NEAR(out(2, 3), 13.0f * -100.0f, 1e-5); 
    EXPECT_NEAR(out(1, 3), 23.0f * -100.0f, 1e-5); 
}

TEST(TransformationTest, SourceCppRegressionCase2) {
    TransformationMeta source_2(
        { Axis::Y, AxisDirection::NEGATIVE },
        { Axis::Z, AxisDirection::NEGATIVE },
        { Axis::X, AxisDirection::POSITIVE }
    );
    EXPECT_FALSE(source_2.isLeftHanded());
}

TEST(TransformationTest, SourceCppRegressionCase3) {
    TransformationMeta source_3 =
    {
        { Axis::Y, AxisDirection::NEGATIVE },
        { Axis::X, AxisDirection::POSITIVE },
        { Axis::Z, AxisDirection::POSITIVE }
    };

    TransformationMeta unity =
    {
        { Axis::X, AxisDirection::POSITIVE },
        { Axis::Z, AxisDirection::POSITIVE },
        { Axis::Y, AxisDirection::POSITIVE }
    };

    TransformationConverter conv(source_3, unity);

    Eigen::Vector3f point { 1, 2, 3 };
    Eigen::Vector3f out_point;
    conv.convert_point(point, out_point);
    
    EXPECT_NEAR(out_point.x(), -2.0f, 1e-5);
    EXPECT_NEAR(out_point.y(), 3.0f, 1e-5);
    EXPECT_NEAR(out_point.z(), 1.0f, 1e-5);
}

TEST(TransformationTest, DoublePrecision) {
    TransformationMeta origin(
        {Axis::X, AxisDirection::POSITIVE},
        {Axis::Y, AxisDirection::POSITIVE},
        {Axis::Z, AxisDirection::POSITIVE}
    );
    TransformationMeta target(
        {Axis::Y, AxisDirection::POSITIVE},
        {Axis::X, AxisDirection::POSITIVE},
        {Axis::Z, AxisDirection::POSITIVE}
    );

    TransformationConverter<double> conv(origin, target);
    
    Eigen::Vector3d point(1.23456789012345, 9.87654321098765, 0.0);
    Eigen::Vector3d out;

    conv.convert_point(point, out);

    EXPECT_DOUBLE_EQ(out.x(), point.y());
    EXPECT_DOUBLE_EQ(out.y(), point.x());
    EXPECT_DOUBLE_EQ(out.z(), point.z());
}

TEST(TransformationTest, BatchConversion) {
    TransformationMeta origin = TransformationMetaBuilder().build();
    TransformationMeta target = TransformationMetaBuilder()
        .right(Axis::Y, AxisDirection::POSITIVE)
        .forward(Axis::X, AxisDirection::POSITIVE)
        .scale(1, 10) // 10x scale
        .build();

    TransformationConverter conv(origin, target);

    std::vector<Eigen::Vector3f> points = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    std::vector<Eigen::Vector3f> out(3);

    conv.convert_points<Eigen::Vector3f, Eigen::Vector3f>(points, out);

    // Swap X/Y and 10x scale
    EXPECT_NEAR(out[0].y(), 10.0f, 1e-5);
    EXPECT_NEAR(out[1].x(), 10.0f, 1e-5);
    EXPECT_NEAR(out[2].z(), 10.0f, 1e-5);
}
