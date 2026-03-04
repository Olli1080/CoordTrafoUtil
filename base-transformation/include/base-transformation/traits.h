#pragma once
#include <cstddef>
#include <type_traits>

namespace Transformation
{
    /** @brief Helper to delay static_assert evaluation until template instantiation. */
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
}
