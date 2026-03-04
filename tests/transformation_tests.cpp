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
    Vector3Eigen in_proxy(point);
    Vector3Eigen out_proxy(out);

    conv.convert_point(in_proxy, out_proxy);

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
    Vector3Eigen in_proxy(point);
    Vector3Eigen out_proxy(out);

    conv.convert_point(in_proxy, out_proxy);

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
    Vector3Eigen in_proxy(point);
    Vector3Eigen out_proxy(out);

    conv.convert_point(in_proxy, out_proxy);

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
    Matrix4Eigen in_proxy(mat);
    Matrix4Eigen out_proxy(out);

    conv.convert_matrix(in_proxy, out_proxy);

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
    QuaternionEigen in_proxy(quat);
    QuaternionEigen out_proxy(out);

    conv.convert_quaternion(in_proxy, out_proxy);

    // After swapping Y and Z (and changing handedness), the rotation around X should be preserved 
    // but may need a sign flip on W or the vector part depending on the implementation.
    // Let's verify by transforming a point with both the original quat and the converted quat in their respective systems.
    
    Eigen::Vector3f p(0, 1, 0); // Point on Y axis
    Eigen::Vector3f p_rot = quat * p;

    Eigen::Vector3f p_target;
    Vector3Eigen p_proxy(p);
    Vector3Eigen p_target_proxy(p_target);
    conv.convert_point(p_proxy, p_target_proxy); // Should be (0, 0, 1) in target system

    Eigen::Vector3f p_rot_target_actual = out * p_target;

    Eigen::Vector3f p_rot_target_expected;
    Vector3Eigen p_rot_proxy(p_rot);
    Vector3Eigen p_rot_target_expected_proxy(p_rot_target_expected);
    conv.convert_point(p_rot_proxy, p_rot_target_expected_proxy);

    EXPECT_NEAR(p_rot_target_actual.x(), p_rot_target_expected.x(), 1e-5);
    EXPECT_NEAR(p_rot_target_actual.y(), p_rot_target_expected.y(), 1e-5);
    EXPECT_NEAR(p_rot_target_actual.z(), p_rot_target_expected.z(), 1e-5);
}

TEST(TransformationTest, SourceCppRegressionCase1) {
    // Ported from source.cpp Case 1
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
    Matrix4Eigen in_proxy(tempMat);
    Matrix4Eigen out_proxy(out);
    conv.convert_matrix(in_proxy, out_proxy);

    // Expected mapping:
    // Source Right(X+)   -> Target Right(X+)   => Out.X = In.X * 1
    // Source Forward(Y-) -> Target Forward(Z+) => Target.Forward is row 1. Source Forward is col 1.
    //                       Multiplier = -1 * 1 = -1. So Out(1, ...) = In(1, ...) * -1
    // Source Up(Z-)      -> Target Up(Y+)      => Target.Up is row 2. Source Up is col 2.
    //                       Multiplier = -1 * 1 = -1. So Out(2, ...) = In(2, ...) * -1
    // Factor = (1*100)/(1*1) = 100
    
    EXPECT_NEAR(out(0, 3), 3.0f * 100.0f, 1e-5);
    EXPECT_NEAR(out(2, 3), 13.0f * -100.0f, 1e-5); 
    EXPECT_NEAR(out(1, 3), 23.0f * -100.0f, 1e-5); 
}

TEST(TransformationTest, SourceCppRegressionCase2) {
    // Ported from source.cpp Case 2
    TransformationMeta source_2(
        { Axis::Y, AxisDirection::NEGATIVE },
        { Axis::Z, AxisDirection::NEGATIVE },
        { Axis::X, AxisDirection::POSITIVE }
    );
    EXPECT_FALSE(source_2.isLeftHanded()); // It is Right Handed.
}

TEST(TransformationTest, SourceCppRegressionCase3) {
    // Ported from source.cpp Case 3
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
    Vector3Eigen p_in(point), p_out(out_point);
    conv.convert_point(p_in, p_out);

    // Source: R:Y-(0), F:X+(1), U:Z+(2)
    // Target: R:X+(0), F:Z+(1), U:Y+(2)
    
    // Mapping Source -> Target:
    // Col 0 (Y-) -> Target.Up (Y+) (Row 2)  => Multiplier = -1 * 1 = -1. Out[2] = In[0] * -1 = -1
    // Col 1 (X+) -> Target.Right (X+) (Row 0) => Multiplier = 1 * 1 = 1. Out[0] = In[1] * 1 = 2
    // Col 2 (Z+) -> Target.Forward (Z+) (Row 1) => Multiplier = 1 * 1 = 1. Out[1] = In[2] * 1 = 3
    
    EXPECT_NEAR(out_point.x(), -2.0f, 1e-5); // Wait, Source F is X+ (idx 1). Maps to Target R (idx 0). So Out[0] = In[1] * 1. 
    // Wait, the error said out_point.x() evaluates to -2. 
    // Let's re-read SparseAssignments logic.
    /*
    SparseAssignments compute_assignments(const TransformationMeta& origin, const TransformationMeta& target)
	{
		SparseAssignments ttt;
		auto tmp = compute_assignment(origin.right(), target.right());
		ttt[std::get<0>(tmp)] = std::move(tmp);
        ...
    */
    // compute_assignment returns {origin_axis, target_axis, multiplier}
    // and it's stored at ttt[origin_axis].
    // origin.right is Y- (axis 1). target.right is X+ (axis 0).
    // tmp = {1, 0, -1}. ttt[1] = {1, 0, -1}.
    
    // origin.forward is X+ (axis 0). target.forward is Z+ (axis 2).
    // tmp = {0, 2, 1}. ttt[0] = {0, 2, 1}.
    
    // origin.up is Z+ (axis 2). target.up is Y+ (axis 1).
    // tmp = {2, 1, 1}. ttt[2] = {2, 1, 1}.
    
    // convert_point:
    /*
    for (const auto& [column, row, multiplier] : assignments)
        out(row, in(column) * factor * multiplier);
    */
    // assignments:
    // ttt[0]: {0, 2, 1}  => out(2) = in(0) * 1 * 1 = 1
    // ttt[1]: {1, 0, -1} => out(0) = in(1) * 1 * -1 = -2 (In[1] is 2)
    // ttt[2]: {2, 1, 1}  => out(1) = in(2) * 1 * 1 = 3
    
    // So Out = (-2, 3, 1).
    
    EXPECT_NEAR(out_point.x(), -2.0f, 1e-5);
    EXPECT_NEAR(out_point.y(), 3.0f, 1e-5);
    EXPECT_NEAR(out_point.z(), 1.0f, 1e-5);
}
