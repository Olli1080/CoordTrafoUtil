#include <gtest/gtest.h>
#include "base-transformation/TransformationHelper.h"
#include <vector>
#include <Eigen/Dense>

using namespace Transformation;

// Helper to convert Axis/Direction to Eigen Vector3f
Eigen::Vector3f to_vector(AxisAlignment aa) {
    Eigen::Vector3f v = Eigen::Vector3f::Zero();
    v[static_cast<int>(aa.axis)] = static_cast<float>(aa.direction);
    return v;
}

// Reference handedness check using cross product: (Right x Forward) . Up > 0
bool reference_is_right_handed(const TransformationMeta& meta) {
    Eigen::Vector3f r = to_vector(meta.right());
    Eigen::Vector3f f = to_vector(meta.forward());
    Eigen::Vector3f u = to_vector(meta.up());
    return r.cross(f).dot(u) > 0.0f;
}

TEST(HandednessTest, ExhaustiveCheck) {
    std::vector<Axis> axes = {Axis::X, Axis::Y, Axis::Z};
    std::vector<AxisDirection> directions = {AxisDirection::POSITIVE, AxisDirection::NEGATIVE};

    int count = 0;
    // Permute axes
    for (auto r_ax : axes) {
        for (auto f_ax : axes) {
            if (f_ax == r_ax) continue;
            for (auto u_ax : axes) {
                if (u_ax == r_ax || u_ax == f_ax) continue;

                // Combine directions
                for (auto r_dir : directions) {
                    for (auto f_dir : directions) {
                        for (auto u_dir : directions) {
                            TransformationMeta meta(
                                {r_ax, r_dir},
                                {f_ax, f_dir},
                                {u_ax, u_dir}
                            );

                            bool expected = reference_is_right_handed(meta);
                            bool actual = meta.isRightHanded();

                            EXPECT_EQ(actual, expected) 
                                << "Failed for axes: " 
                                << static_cast<int>(r_ax) << (r_dir == AxisDirection::POSITIVE ? "+" : "-") << ", "
                                << static_cast<int>(f_ax) << (f_dir == AxisDirection::POSITIVE ? "+" : "-") << ", "
                                << static_cast<int>(u_ax) << (u_dir == AxisDirection::POSITIVE ? "+" : "-");
                            
                            count++;
                        }
                    }
                }
            }
        }
    }
    EXPECT_EQ(count, 48); // 3! * 2^3 = 6 * 8 = 48
}

TEST(HandednessTest, StandardSystems) {
    // Unity: Right (+X), Up (+Y), Forward (+Z) -> Left-handed (often interpreted differently, but let's check our meta)
    // Wait, Unity is typically +X Right, +Y Up, +Z Forward.
    // Cross product: X x Z = -Y. Dot Y = -1. So Left-handed.
    TransformationMeta unity({Axis::X, AxisDirection::POSITIVE}, {Axis::Z, AxisDirection::POSITIVE}, {Axis::Y, AxisDirection::POSITIVE});
    EXPECT_TRUE(unity.isLeftHanded());

    // Unreal: Right (+Y), Forward (+X), Up (+Z) -> Left-handed
    // Cross product: Y x X = -Z. Dot Z = -1. So Left-handed.
    TransformationMeta unreal({Axis::Y, AxisDirection::POSITIVE}, {Axis::X, AxisDirection::POSITIVE}, {Axis::Z, AxisDirection::POSITIVE});
    EXPECT_TRUE(unreal.isLeftHanded());

    // Standard Right-Handed: Right (+X), Forward (+Y), Up (+Z)
    // Cross product: X x Y = Z. Dot Z = 1.
    TransformationMeta standard_rh({Axis::X, AxisDirection::POSITIVE}, {Axis::Y, AxisDirection::POSITIVE}, {Axis::Z, AxisDirection::POSITIVE});
    EXPECT_TRUE(standard_rh.isRightHanded());
}
