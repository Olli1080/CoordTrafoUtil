#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "base-transformation/traits.h"

namespace Transformation
{
    // Eigen::Vector3f specialization
    template<>
    struct VectorTraits<Eigen::Vector3f, float> {
        static float get_x(const Eigen::Vector3f& v) { return v.x(); }
        static float get_y(const Eigen::Vector3f& v) { return v.y(); }
        static float get_z(const Eigen::Vector3f& v) { return v.z(); }
        static float get_idx(const Eigen::Vector3f& v, size_t i) { return v(i); }

        static void set_x(Eigen::Vector3f& v, float val) { v.x() = val; }
        static void set_y(Eigen::Vector3f& v, float val) { v.y() = val; }
        static void set_z(Eigen::Vector3f& v, float val) { v.z() = val; }
        static void set_idx(Eigen::Vector3f& v, size_t i, float val) { v(i) = val; }
    };

    // Eigen::Matrix3f specialization
    template<>
    struct MatrixTraits<Eigen::Matrix3f, float> {
        static constexpr size_t size = 3;
        static float get(const Eigen::Matrix3f& m, size_t r, size_t c) { return m(r, c); }
        static void set(Eigen::Matrix3f& m, size_t r, size_t c, float val) { m(r, c) = val; }
    };

    // Eigen::Matrix4f specialization
    template<>
    struct MatrixTraits<Eigen::Matrix4f, float> {
        static constexpr size_t size = 4;
        static float get(const Eigen::Matrix4f& m, size_t r, size_t c) { return m(r, c); }
        static void set(Eigen::Matrix4f& m, size_t r, size_t c, float val) { m(r, c) = val; }
    };

    // Eigen::Quaternionf specialization
    template<>
    struct QuaternionTraits<Eigen::Quaternionf, float> {
        static float get_x(const Eigen::Quaternionf& q) { return q.x(); }
        static float get_y(const Eigen::Quaternionf& q) { return q.y(); }
        static float get_z(const Eigen::Quaternionf& q) { return q.z(); }
        static float get_w(const Eigen::Quaternionf& q) { return q.w(); }
        static float get_idx(const Eigen::Quaternionf& q, size_t i) { return q.coeffs()(i); }

        static void set_x(Eigen::Quaternionf& q, float val) { q.x() = val; }
        static void set_y(Eigen::Quaternionf& q, float val) { q.y() = val; }
        static void set_z(Eigen::Quaternionf& q, float val) { q.z() = val; }
        static void set_w(Eigen::Quaternionf& q, float val) { q.w() = val; }
        static void set_idx(Eigen::Quaternionf& q, size_t i, float val) { q.coeffs()(i) = val; }
    };
}
