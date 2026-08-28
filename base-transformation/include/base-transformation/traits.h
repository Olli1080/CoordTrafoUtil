#pragma once
#include <cstddef>
#include <type_traits>

namespace Transformation
{
    /** 
     * @brief Helper to delay static_assert evaluation until template instantiation. 
     * 
     * By default, this is set to false. When you specialize a Trait (e.g., VectorTraits) 
     * for a custom type, you MUST also specialize TraitsEnabled<T> to std::true_type.
     * This signals to the library that the traits are valid and suppresses compile-time 
     * "missing specialization" errors.
     */
    template<typename T> struct TraitsEnabled : std::false_type {};

    /**
     * @brief Customization point for vector-like types.
     * @tparam T The vector wrapper or math type.
     * @tparam ValueType The scalar type.
     */
    template<typename T, typename ValueType>
    struct VectorTraits {
        static_assert(TraitsEnabled<T>::value, 
            "CoordTrafoUtil Error: VectorTraits not specialized for this type. "
            "Please specialize Transformation::VectorTraits<T, ValueType>.");
    };

    /**
     * @brief Customization point for matrix-like types.
     */
    template<typename T, typename ValueType>
    struct MatrixTraits {
        static_assert(TraitsEnabled<T>::value, 
            "CoordTrafoUtil Error: MatrixTraits not specialized for this type. "
            "Please specialize Transformation::MatrixTraits<T, ValueType>.");
    };

    /**
     * @brief Customization point for quaternion-like types.
     */
    template<typename T, typename ValueType>
    struct QuaternionTraits {
        static_assert(TraitsEnabled<T>::value,
            "CoordTrafoUtil Error: QuaternionTraits not specialized for this type. "
            "Please specialize Transformation::QuaternionTraits<T, ValueType>.");
    };

    /**
     * @brief CRTP mix-in that synthesises get_x/y/z and set_x/y/z from a
     *        Derived::get_idx / Derived::set_idx pair.
     *
     * A VectorTraits specialisation only needs to provide `type`, `get_idx` and
     * `set_idx`; inherit from this to get the named component accessors for free:
     * @code
     * template<> struct VectorTraits<MyVec, double>
     *     : VectorAccessByIndex<VectorTraits<MyVec, double>, double> {
     *     using type = MyVec;
     *     static double get_idx(const MyVec& v, size_t i) { return v.d[i]; }
     *     static void   set_idx(MyVec& v, size_t i, double val) { v.d[i] = val; }
     * };
     * @endcode
     */
    template<typename Derived, typename ValueType>
    struct VectorAccessByIndex {
        static ValueType get_x(const auto& v) { return Derived::get_idx(v, 0); }
        static ValueType get_y(const auto& v) { return Derived::get_idx(v, 1); }
        static ValueType get_z(const auto& v) { return Derived::get_idx(v, 2); }
        static void set_x(auto& v, ValueType val) { Derived::set_idx(v, 0, val); }
        static void set_y(auto& v, ValueType val) { Derived::set_idx(v, 1, val); }
        static void set_z(auto& v, ValueType val) { Derived::set_idx(v, 2, val); }
    };

    /**
     * @brief CRTP mix-in that synthesises get_x/y/z/w and set_x/y/z/w from a
     *        Derived::get_idx / Derived::set_idx pair (w at index 3, order [x,y,z,w]).
     */
    template<typename Derived, typename ValueType>
    struct QuaternionAccessByIndex {
        static ValueType get_x(const auto& q) { return Derived::get_idx(q, 0); }
        static ValueType get_y(const auto& q) { return Derived::get_idx(q, 1); }
        static ValueType get_z(const auto& q) { return Derived::get_idx(q, 2); }
        static ValueType get_w(const auto& q) { return Derived::get_idx(q, 3); }
        static void set_x(auto& q, ValueType val) { Derived::set_idx(q, 0, val); }
        static void set_y(auto& q, ValueType val) { Derived::set_idx(q, 1, val); }
        static void set_z(auto& q, ValueType val) { Derived::set_idx(q, 2, val); }
        static void set_w(auto& q, ValueType val) { Derived::set_idx(q, 3, val); }
    };
}
