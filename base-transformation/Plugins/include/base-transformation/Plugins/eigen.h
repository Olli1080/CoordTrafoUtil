#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <base-transformation/traits.h>

namespace Transformation
{
    template<typename T>        struct TraitsEnabled<Eigen::Vector3<T>>      : std::true_type {};
    template<typename T, int N> struct TraitsEnabled<Eigen::Matrix<T, N, N>> : std::true_type {};
    template<typename T>        struct TraitsEnabled<Eigen::Quaternion<T>>   : std::true_type {};

    template<typename T>
    struct VectorTraits<Eigen::Vector3<T>, T>
        : VectorAccessByIndex<VectorTraits<Eigen::Vector3<T>, T>, T>
    {
        using type = Eigen::Vector3<T>;
        static T get_idx(const type& v, size_t i) { return v(static_cast<Eigen::Index>(i)); }
        static void set_idx(type& v, size_t i, T val) { v(static_cast<Eigen::Index>(i)) = val; }
    };

    // Covers Eigen::Matrix3<T> (== Matrix<T,3,3>) and Eigen::Matrix4<T> (== Matrix<T,4,4>).
    template<typename T, int N>
    struct MatrixTraits<Eigen::Matrix<T, N, N>, T>
    {
        using type = Eigen::Matrix<T, N, N>;
        static constexpr size_t size = static_cast<size_t>(N);
        static T get(const type& m, size_t r, size_t c) {
            return m(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c));
        }
        static void set(type& m, size_t r, size_t c, T val) {
            m(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = val;
        }
    };

    template<typename T>
    struct QuaternionTraits<Eigen::Quaternion<T>, T>
        : QuaternionAccessByIndex<QuaternionTraits<Eigen::Quaternion<T>, T>, T>
    {
        using type = Eigen::Quaternion<T>;
        // Eigen quaternion coefficients are stored [x, y, z, w].
        static T get_idx(const type& q, size_t i) { return q.coeffs()(static_cast<Eigen::Index>(i)); }
        static void set_idx(type& q, size_t i, T val) { q.coeffs()(static_cast<Eigen::Index>(i)) = val; }
    };
}
