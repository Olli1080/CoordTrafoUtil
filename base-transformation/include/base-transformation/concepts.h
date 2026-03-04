#pragma once
#include <concepts>
#include <base-transformation/traits.h>

namespace Transformation
{
    /**
     * @brief Concept for types that provide read access to matrix components via MatrixTraits.
     */
    template<typename T, typename ValueType>
    concept matrix_getter = requires(const T m, size_t row, size_t col)
    {
        { MatrixTraits<T, ValueType>::get(m, row, col) } -> std::same_as<ValueType>;
        { MatrixTraits<T, ValueType>::size } -> std::convertible_to<size_t>;
    };

    /**
     * @brief Concept for types that provide write access to matrix components via MatrixTraits.
     */
    template<typename T, typename ValueType>
    concept matrix_setter = requires(T m, size_t row, size_t col, ValueType val)
    {
        { MatrixTraits<T, ValueType>::set(m, row, col, val) } -> std::same_as<void>;
    };

    /**
     * @brief Concept for types that provide constant (read-only) access to matrix components.
     */
    template<typename T, typename ValueType>
    concept matrix_const_access = matrix_getter<T, ValueType>;

    /**
     * @brief Concept for types that provide full (read/write) access to matrix components.
     */
    template<typename T, typename ValueType>
    concept matrix_full_access = matrix_const_access<T, ValueType> and matrix_setter<T, ValueType>;

    /**
     * @brief Concept for types that provide read access to vector components via VectorTraits.
     */
    template<typename T, typename ValueType>
    concept vector_getter = requires(const T v, size_t idx)
    {
        { VectorTraits<T, ValueType>::get_x(v) } -> std::same_as<ValueType>;
        { VectorTraits<T, ValueType>::get_y(v) } -> std::same_as<ValueType>;
        { VectorTraits<T, ValueType>::get_z(v) } -> std::same_as<ValueType>;
        { VectorTraits<T, ValueType>::get_idx(v, idx) } -> std::same_as<ValueType>;
    };

    /**
     * @brief Concept for types that provide write access to vector components via VectorTraits.
     */
    template<typename T, typename ValueType>
    concept vector_setter = requires(T v, size_t idx, ValueType value)
    {
        { VectorTraits<T, ValueType>::set_x(v, value) } -> std::same_as<void>;
        { VectorTraits<T, ValueType>::set_y(v, value) } -> std::same_as<void>;
        { VectorTraits<T, ValueType>::set_z(v, value) } -> std::same_as<void>;
        { VectorTraits<T, ValueType>::set_idx(v, idx, value) } -> std::same_as<void>;
    };

    /**
     * @brief Concept for types that provide constant (read-only) access to vector components.
     */
    template<typename T, typename ValueType>
    concept vector_const_access = vector_getter<T, ValueType>;

    /**
     * @brief Concept for types that provide full (read/write) access to vector components.
     */
    template<typename T, typename ValueType>
    concept vector_full_access = vector_const_access<T, ValueType> and vector_setter<T, ValueType>;

    /**
     * @brief Concept for types that provide read access to quaternion components via QuaternionTraits.
     */
    template<typename T, typename ValueType>
    concept quaternion_getter = requires(const T q, size_t idx)
    {
        { QuaternionTraits<T, ValueType>::get_x(q) } -> std::same_as<ValueType>;
        { QuaternionTraits<T, ValueType>::get_y(q) } -> std::same_as<ValueType>;
        { QuaternionTraits<T, ValueType>::get_z(q) } -> std::same_as<ValueType>;
        { QuaternionTraits<T, ValueType>::get_w(q) } -> std::same_as<ValueType>;
        { QuaternionTraits<T, ValueType>::get_idx(q, idx) } -> std::same_as<ValueType>;
    };

    /**
     * @brief Concept for types that provide write access to quaternion components via QuaternionTraits.
     */
    template<typename T, typename ValueType>
    concept quaternion_setter = requires(T q, size_t idx, ValueType value)
    {
        { QuaternionTraits<T, ValueType>::set_x(q, value) } -> std::same_as<void>;
        { QuaternionTraits<T, ValueType>::set_y(q, value) } -> std::same_as<void>;
        { QuaternionTraits<T, ValueType>::set_z(q, value) } -> std::same_as<void>;
        { QuaternionTraits<T, ValueType>::set_w(q, value) } -> std::same_as<void>;
        { QuaternionTraits<T, ValueType>::set_idx(q, idx, value) } -> std::same_as<void>;
    };

    /**
     * @brief Concept for types that provide constant (read-only) access to quaternion components.
     */
    template<typename T, typename ValueType>
    concept quaternion_const_access = quaternion_getter<T, ValueType>;

    /**
     * @brief Concept for types that provide full (read/write) access to quaternion components.
     */
    template<typename T, typename ValueType>
    concept quaternion_full_access = quaternion_const_access<T, ValueType> and quaternion_setter<T, ValueType>;
}
