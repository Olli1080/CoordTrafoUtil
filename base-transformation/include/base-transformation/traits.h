#pragma once
#include <cstddef>

namespace Transformation
{
    // Customization points for different mathematical types.
    // Users can specialize these templates for their own types.

    template<typename T, typename ValueType = float>
    struct VectorTraits {
        // static ValueType get(const T& v, size_t i);
        // static void set(T& v, size_t i, ValueType val);
    };

    template<typename T, typename ValueType = float>
    struct MatrixTraits {
        // static constexpr size_t size = ...;
        // static ValueType get(const T& m, size_t r, size_t c);
        // static void set(T& m, size_t r, size_t c, ValueType val);
    };

    template<typename T, typename ValueType = float>
    struct QuaternionTraits {
        // static ValueType get_x(const T& q);
        // static ValueType get_y(const T& q);
        // static ValueType get_z(const T& q);
        // static ValueType get_w(const T& q);
        // static void set_x(T& q, ValueType val);
        // ...
        // static ValueType get_idx(const T& q, size_t i);
        // static void set_idx(T& q, size_t i, ValueType val);
    };
}
