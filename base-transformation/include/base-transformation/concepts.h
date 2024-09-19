#pragma once
#include <concepts>

template<typename T, typename ValueType>
concept matrix_getter = requires(const T v1, size_t row, size_t column)
{
	{ v1.operator()(row, column) } -> std::same_as<const ValueType&>;
};

template<typename T, typename ValueType>
concept matrix_setter = requires(T v0, size_t row, size_t column)
{
	{ v0.operator()(row, column) } -> std::same_as<ValueType&>;
};

template<typename T, typename ValueType>
concept matrix_const_access = matrix_getter<T, ValueType> and requires
{
	typename T::type;
	{ std::same_as<decltype(T::size), size_t> };
};

template<typename T, typename ValueType>
concept matrix_full_access = matrix_const_access<T, ValueType> and matrix_setter<T, ValueType>;

/*template<typename T, typename ValueType>
concept matrix_access = requires(T v0, const T v1, size_t row, size_t column)
{
	typename T::type;

	{ v0.operator()(row, column) } -> std::same_as<ValueType&>;
	{ v1.operator()(row, column) } -> std::same_as<const ValueType&>;

	{ std::same_as<decltype(T::size), size_t> };
};*/
template<typename T, typename ValueType>
concept vector_getter = requires(const T v1, size_t idx)
{
	{ v1.get_x() } -> std::same_as<ValueType>;
	{ v1.get_y() } -> std::same_as<ValueType>;
	{ v1.get_z() } -> std::same_as<ValueType>;

	{ v1.operator()(idx) } -> std::same_as<ValueType>;
};

template<typename T, typename ValueType>
concept vector_setter = requires(T v0, size_t idx, ValueType value)
{
	{ v0.set_x(value) } -> std::same_as<void>;
	{ v0.set_y(value) } -> std::same_as<void>;
	{ v0.set_z(value) } -> std::same_as<void>;

	{ v0.operator()(idx, value) } -> std::same_as<void>;
};

template<typename T, typename ValueType>
concept vector_const_access = vector_getter<T, ValueType> and requires
{
	typename T::type;
};

template<typename T, typename ValueType>
concept vector_full_access = vector_const_access<T, ValueType> and vector_setter<T, ValueType>;

/*
template<typename T, typename ValueType>
concept vector_access = requires(T v0, const T v1, size_t idx, ValueType value)
{
	typename T::type;

	{ v0.set_x(value) } -> std::same_as<void>;
	{ v0.set_y(value) } -> std::same_as<void>;
	{ v0.set_z(value) } -> std::same_as<void>;

	{ v1.get_x() } -> std::same_as<ValueType>;
	{ v1.get_y() } -> std::same_as<ValueType>;
	{ v1.get_z() } -> std::same_as<ValueType>;

	{ v0.operator()(idx, value) } -> std::same_as<void>;
	{ v1.operator()(idx) } -> std::same_as<ValueType>;
};*/

template<typename T, typename ValueType>
concept quaternion_getter = requires(const T v1, size_t idx)
{
	{ v1.get_x() } -> std::same_as<ValueType>;
	{ v1.get_y() } -> std::same_as<ValueType>;
	{ v1.get_z() } -> std::same_as<ValueType>;
	{ v1.get_w() } -> std::same_as<ValueType>;

	//assumes [x,y,z, w]
	{ v1.operator()(idx) } -> std::same_as<ValueType>;
};

template<typename T, typename ValueType>
concept quaternion_setter = requires(T v0, size_t idx, ValueType value)
{
	{ v0.set_x(value) } -> std::same_as<void>;
	{ v0.set_y(value) } -> std::same_as<void>;
	{ v0.set_z(value) } -> std::same_as<void>;
	{ v0.set_w(value) } -> std::same_as<void>;

	//assumes [x,y,z, w]
	{ v0.operator()(idx, value) } -> std::same_as<void>;
};

template<typename T, typename ValueType>
concept quaternion_const_access = quaternion_getter<T, ValueType> and requires
{
	typename T::type;
};

template<typename T, typename ValueType>
concept quaternion_full_access = quaternion_const_access<T, ValueType> and quaternion_setter<T, ValueType>;

/*
template<typename T, typename ValueType>
concept quaternion_access = requires(T v0, const T v1, size_t idx, ValueType value)
{
	typename T::type;

	{ v0.set_x(value) } -> std::same_as<void>;
	{ v0.set_y(value) } -> std::same_as<void>;
	{ v0.set_z(value) } -> std::same_as<void>;
	{ v0.set_w(value) } -> std::same_as<void>;

	{ v1.get_x() } -> std::same_as<ValueType>;
	{ v1.get_y() } -> std::same_as<ValueType>;
	{ v1.get_z() } -> std::same_as<ValueType>;
	{ v1.get_w() } -> std::same_as<ValueType>;

	//assumes [x,y,z, w]
	{ v0.operator()(idx, value) } -> std::same_as<void>;
	{ v1.operator()(idx) } -> std::same_as<ValueType>;
};*/