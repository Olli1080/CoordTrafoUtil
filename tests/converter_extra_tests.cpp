#include <gtest/gtest.h>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <base-transformation/TransformationHelper.h>
#include <base-transformation/io.h>
#include <base-transformation/Plugins/eigen.h>

using namespace Transformation;

namespace {

TransformationMeta rh_std() {
    return { {Axis::X, AxisDirection::POSITIVE},
             {Axis::Y, AxisDirection::POSITIVE},
             {Axis::Z, AxisDirection::POSITIVE} };
}

} // namespace

// ---------------------------------------------------------------------------
// inverse()
// ---------------------------------------------------------------------------
TEST(ConverterExtra, InverseRoundTripsPoint) {
    TransformationMeta o(rh_std());
    TransformationMeta t({Axis::Y, AxisDirection::NEGATIVE},
                         {Axis::Z, AxisDirection::POSITIVE},
                         {Axis::X, AxisDirection::POSITIVE},
                         {1, 100});

    TransformationConverter conv(o, t);
    auto back = conv.inverse();

    EXPECT_EQ(back.origin(), t);
    EXPECT_EQ(back.target(), o);

    Eigen::Vector3f p(1.5f, -2.0f, 3.25f);
    Eigen::Vector3f there, roundtrip;
    conv.convert_point(p, there);
    back.convert_point(there, roundtrip);

    EXPECT_NEAR(roundtrip.x(), p.x(), 1e-4);
    EXPECT_NEAR(roundtrip.y(), p.y(), 1e-4);
    EXPECT_NEAR(roundtrip.z(), p.z(), 1e-4);
}

// ---------------------------------------------------------------------------
// then() composition
// ---------------------------------------------------------------------------
TEST(ConverterExtra, ThenMatchesSequentialApplication) {
    TransformationMeta a(rh_std());
    TransformationMeta mid({Axis::Y, AxisDirection::POSITIVE},
                           {Axis::X, AxisDirection::POSITIVE},
                           {Axis::Z, AxisDirection::POSITIVE},
                           {1, 10});
    TransformationMeta c({Axis::X, AxisDirection::POSITIVE},
                         {Axis::Z, AxisDirection::POSITIVE},
                         {Axis::Y, AxisDirection::POSITIVE},
                         {1, 20});

    TransformationConverter ab(a, mid);
    TransformationConverter bc(mid, c);
    auto ac = ab.then(bc);

    Eigen::Vector3f p(2.0f, -1.0f, 4.0f);

    Eigen::Vector3f seq_mid, seq_end;
    ab.convert_point(p, seq_mid);
    bc.convert_point(seq_mid, seq_end);

    Eigen::Vector3f direct;
    ac.convert_point(p, direct);

    EXPECT_NEAR(direct.x(), seq_end.x(), 1e-4);
    EXPECT_NEAR(direct.y(), seq_end.y(), 1e-4);
    EXPECT_NEAR(direct.z(), seq_end.z(), 1e-4);
}

TEST(ConverterExtra, ThenRejectsMismatchedFrames) {
    TransformationConverter ab(rh_std(), Presets::Unity());
    TransformationConverter bc(Presets::Unreal(), Presets::OpenGL());
    EXPECT_THROW((void)ab.then(bc), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// 3x3 (rotation-only) convert_matrix
// ---------------------------------------------------------------------------
TEST(ConverterExtra, Convert3x3IsLinearOnly) {
    TransformationMeta o(rh_std());
    // swap Y and Z
    TransformationMeta t({Axis::X, AxisDirection::POSITIVE},
                         {Axis::Z, AxisDirection::POSITIVE},
                         {Axis::Y, AxisDirection::POSITIVE},
                         {1, 100}); // scale must NOT affect a 3x3

    TransformationConverter conv(o, t);

    Eigen::Matrix3f in;
    in << 1, 2, 3,
          4, 5, 6,
          7, 8, 9;

    Eigen::Matrix3f out;
    conv.convert_matrix(in, out);

    // Rows/cols 1 and 2 are swapped; no scaling.
    Eigen::Matrix3f expected;
    expected << 1, 3, 2,
                7, 9, 8,
                4, 6, 5;

    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            EXPECT_NEAR(out(r, c), expected(r, c), 1e-5) << "at (" << r << "," << c << ")";
}

// ---------------------------------------------------------------------------
// convert_size: axis re-basing + scale, but never a sign flip
// ---------------------------------------------------------------------------
TEST(ConverterExtra, ConvertSizeIgnoresDirection) {
    TransformationMeta o({Axis::X, AxisDirection::NEGATIVE},
                         {Axis::Y, AxisDirection::NEGATIVE},
                         {Axis::Z, AxisDirection::POSITIVE});
    TransformationMeta t({Axis::X, AxisDirection::POSITIVE},
                         {Axis::Y, AxisDirection::POSITIVE},
                         {Axis::Z, AxisDirection::POSITIVE},
                         {1, 100});

    TransformationConverter conv(o, t);

    Eigen::Vector3f extent(2.0f, 3.0f, 4.0f);
    Eigen::Vector3f out;
    conv.convert_size(extent, out);

    EXPECT_NEAR(out.x(), 200.0f, 1e-4);
    EXPECT_NEAR(out.y(), 300.0f, 1e-4);
    EXPECT_NEAR(out.z(), 400.0f, 1e-4);
}

TEST(ConverterExtra, ConvertScale) {
    TransformationConverter conv(rh_std(),
        TransformationMeta(rh_std().right(), rh_std().forward(), rh_std().up(), {1, 100}));
    EXPECT_NEAR(conv.convert_scale(2.5f), 250.0f, 1e-4);
}

// ---------------------------------------------------------------------------
// Batch helpers
// ---------------------------------------------------------------------------
TEST(ConverterExtra, BatchSizesMatchScalar) {
    TransformationConverter conv(rh_std(),
        TransformationMeta({Axis::Y, AxisDirection::POSITIVE},
                           {Axis::X, AxisDirection::POSITIVE},
                           {Axis::Z, AxisDirection::POSITIVE}, {1, 4}));

    std::vector<Eigen::Vector3f> in = { {1, 2, 3}, {4, 5, 6} };
    std::vector<Eigen::Vector3f> out(2);
    conv.convert_sizes<Eigen::Vector3f, Eigen::Vector3f>(in, out);

    for (size_t i = 0; i < in.size(); ++i) {
        Eigen::Vector3f ref;
        conv.convert_size(in[i], ref);
        EXPECT_NEAR(out[i].x(), ref.x(), 1e-5);
        EXPECT_NEAR(out[i].y(), ref.y(), 1e-5);
        EXPECT_NEAR(out[i].z(), ref.z(), 1e-5);
    }
}

// ---------------------------------------------------------------------------
// Presets
// ---------------------------------------------------------------------------
TEST(ConverterExtra, PresetsHandedness) {
    EXPECT_TRUE(Presets::Unity().isLeftHanded());
    EXPECT_TRUE(Presets::Unreal().isLeftHanded());
    EXPECT_TRUE(Presets::OpenGL().isRightHanded());
    EXPECT_TRUE(Presets::ROS().isRightHanded());
}

// ---------------------------------------------------------------------------
// Equality + Ratio
// ---------------------------------------------------------------------------
TEST(ConverterExtra, MetaEquality) {
    EXPECT_EQ(Presets::Unity(), Presets::Unity());
    EXPECT_NE(Presets::Unity(), Presets::Unreal());
    // Unequal only through scale.
    TransformationMeta a(rh_std().right(), rh_std().forward(), rh_std().up(), {1, 1});
    TransformationMeta b(rh_std().right(), rh_std().forward(), rh_std().up(), {1, 2});
    EXPECT_NE(a, b);
}

TEST(ConverterExtra, RatioNormalisesAndDefaults) {
    EXPECT_EQ(Ratio(), Ratio(1, 1));
    EXPECT_EQ(Ratio(2, 200), Ratio(1, 100));
    EXPECT_EQ(Ratio(std::centi{}), Ratio(1, 100));
    EXPECT_THROW(Ratio(1, 0), std::invalid_argument);
    EXPECT_THROW(Ratio(-1, 2), std::invalid_argument);
    EXPECT_THROW(Ratio(1, -2), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Text output
// ---------------------------------------------------------------------------
TEST(ConverterExtra, ToStringAndStream) {
    const auto meta = TransformationMetaBuilder()
        .right("X+"_a).forward("Z+"_a).up("Y+"_a).build();

    const std::string s = to_string(meta);
    EXPECT_NE(s.find("R=X+"), std::string::npos);
    EXPECT_NE(s.find("F=Z+"), std::string::npos);
    EXPECT_NE(s.find("U=Y+"), std::string::npos);
    EXPECT_NE(s.find("LEFT"), std::string::npos);

    std::ostringstream os;
    os << Axis::Z << to_string(AxisDirection::NEGATIVE);
    EXPECT_EQ(os.str(), "Z-");
}

// ---------------------------------------------------------------------------
// get_conv_matrix (3x3)
// ---------------------------------------------------------------------------
TEST(ConverterExtra, GetConvMatrix3x3) {
    TransformationConverter conv(rh_std(),
        TransformationMeta({Axis::Y, AxisDirection::POSITIVE},
                           {Axis::X, AxisDirection::POSITIVE},
                           {Axis::Z, AxisDirection::POSITIVE}, {1, 100}));

    auto m = conv.get_conv_matrix<Eigen::Matrix3f>();

    // origin X (col 0) -> target Y (row 1), factor 100
    EXPECT_NEAR(m(1, 0), 100.0f, 1e-4);
    EXPECT_NEAR(m(0, 1), 100.0f, 1e-4);
    EXPECT_NEAR(m(2, 2), 100.0f, 1e-4);
    EXPECT_NEAR(m(0, 0), 0.0f, 1e-4);
}

// ---------------------------------------------------------------------------
// Compile-time construction
// ---------------------------------------------------------------------------
TEST(ConverterExtra, ConstexprConverter) {
    constexpr TransformationMeta gl = Presets::OpenGL();
    static_assert(gl.isRightHanded());

    constexpr TransformationConverter<float> c(Presets::Unity(), Presets::Unreal());
    static_assert(c.convert_scale(1.0f) == 100.0f);
    static_assert(c.origin() == Presets::Unity());
    SUCCEED();
}
