#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <base-transformation/traits.h>

namespace Transformation
{
    template<typename T> struct TraitsEnabled<Eigen::Vector3<T>> : std::true_type {};
    template<typename T> struct TraitsEnabled<Eigen::Matrix3<T>> : std::true_type {};
    template<typename T> struct TraitsEnabled<Eigen::Matrix4<T>> : std::true_type {};
    template<typename T> struct TraitsEnabled<Eigen::Quaternion<T>> : std::true_type {};

    template<typename T>
    struct VectorTraits<Eigen::Vector3<T>, T> {
        using type = Eigen::Vector3<T>;
        static T get_x(const type& v) { return v.x(); }
        static T get_y(const type& v) { return v.y(); }
        static T get_z(const type& v) { return v.z(); }
        static T get_idx(const type& v, size_t i) { return v(i); }
        static void set_x(type& v, T val) { v.x() = val; }
        static void set_y(type& v, T val) { v.y() = val; }
        static void set_z(type& v, T val) { v.z() = val; }
        static void set_idx(type& v, size_t i, T val) { v(i) = val; }
    };

    template<typename T>
    struct MatrixTraits<Eigen::Matrix3<T>, T> {
        using type = Eigen::Matrix3<T>;
        static constexpr size_t size = 3;
        static T get(const type& m, size_t r, size_t c) { return m(r, c); }
        static void set(type& m, size_t r, size_t c, T val) { m(r, c) = val; }
    };

    template<typename T>
    struct MatrixTraits<Eigen::Matrix4<T>, T> {
        using type = Eigen::Matrix4<T>;
        static constexpr size_t size = 4;
        static T get(const type& m, size_t r, size_t c) { return m(r, c); }
        static void set(type& m, size_t r, size_t c, T val) { m(r, c) = val; }
    };

    template<typename T>
    struct QuaternionTraits<Eigen::Quaternion<T>, T> {
        using type = Eigen::Quaternion<T>;
        static T get_x(const type& q) { return q.x(); }
        static T get_y(const type& q) { return q.y(); }
        static T get_z(const type& q) { return q.z(); }
        static T get_w(const type& q) { return q.w(); }
        static T get_idx(const type& q, size_t i) { return q.coeffs()(i); }
        static void set_x(type& q, T val) { q.x() = val; }
        static void set_y(type& q, T val) { q.y() = val; }
        static void set_z(type& q, T val) { q.z() = val; }
        static void set_w(type& q, T val) { q.w() = val; }
        static void set_idx(type& q, size_t i, T val) { q.coeffs()(i) = val; }
    };
}
