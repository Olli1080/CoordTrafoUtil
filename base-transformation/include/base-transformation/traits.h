#pragma once
#include <cstddef>

namespace Transformation
{
    /**
     * @brief Customization point for vector-like types.
     * @tparam T The vector wrapper or math type.
     * @tparam ValueType The scalar type.
     * 
     * Requirements:
     * - using type = T; (The underlying math type)
     * - static ValueType get_x(const T& v); ...
     */
    template<typename T, typename ValueType>
    struct VectorTraits {};

    /**
     * @brief Customization point for matrix-like types.
     * 
     * Requirements:
     * - using type = T;
     * - static constexpr size_t size;
     * - static ValueType get(const T& m, size_t r, size_t c); ...
     */
    template<typename T, typename ValueType>
    struct MatrixTraits {};

    /**
     * @brief Customization point for quaternion-like types.
     * 
     * Requirements:
     * - using type = T;
     * - static ValueType get_x(const T& q); ...
     */
    template<typename T, typename ValueType>
    struct QuaternionTraits {};
}
