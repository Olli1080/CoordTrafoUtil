#pragma once
#include <cstdint>
#include <memory>
#include <ratio>

#include <Eigen/Core>

enum class Axis : int8_t
{
	X = 0,
	Y = 1,
	Z = 2
};

enum class AxisDirection : int8_t
{
	POSITIVE = +1,
	NEGATIVE = -1
};

[[nodiscard]] AxisDirection invert(AxisDirection in);

enum class TransformOperation
{
	RIGHT_AND_FORWARD,
	RIGHT_AND_UP,
	FORWARD_AND_UP
};

struct AxisAlignment
{
	Axis axis;
	AxisDirection direction;
};

struct Ratio
{
	std::intmax_t Num;
	std::intmax_t Denom;

	Ratio(std::intmax_t Num, std::intmax_t Denom);

	template<std::intmax_t Num, std::intmax_t Denom>
	inline Ratio()
		: Num(Num), Denom(Denom)
	{}

	template<std::intmax_t Num, std::intmax_t Denom>
	inline Ratio(std::ratio<Num, Denom> ratio)
		: Num(Num), Denom(Denom)
	{}

	[[nodiscard]] float factor() const;
	[[nodiscard]] float factor(const Ratio& other) const;
};


struct TransformationMeta
{
	TransformationMeta(
		AxisAlignment right,
		AxisAlignment forward,
		AxisAlignment up,
		Ratio scale = {1, 1}
	);

	TransformationMeta(const TransformationMeta& other);

	[[nodiscard]] bool isRightHanded() const;
	[[nodiscard]] bool isLeftHanded() const;

	[[nodiscard]] Eigen::Matrix3f get_conv_matrix(const TransformationMeta& target) const;

	[[nodiscard]] Eigen::Matrix4f convert_matrix(const TransformationMeta& target, const Eigen::Matrix4f& in) const;
	[[nodiscard]] Eigen::Quaternion<float> convert_quaternion(const TransformationMeta& target, const Eigen::Quaternion<float>& in) const;
	[[nodiscard]] Eigen::Vector3f convert_point(const TransformationMeta& target, const Eigen::Vector3f& in) const;

	[[nodiscard]] std::function<Eigen::Matrix4f(const Eigen::Matrix4f&)> generate_convert_function(const TransformationMeta& target) const;

private:

	void rotateLeft();
	void rotateRight();

	static std::tuple<int8_t, int8_t, float> assignment(AxisAlignment axis, AxisAlignment target_axis);

	static std::array<std::tuple<int8_t, int8_t, float>, 3> assignments(const TransformationMeta& origin, const TransformationMeta& target);

	static Eigen::Matrix4f convert(const std::array<std::tuple<int8_t, int8_t, float>, 3>& ttt, const Eigen::Matrix4f& in, float scale);

	AxisAlignment right;
	AxisAlignment forward;
	AxisAlignment up;

	Ratio scale;

	mutable std::unique_ptr<bool> right_handed;
};