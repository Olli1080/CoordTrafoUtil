#include "TransformationHelper.h"

#include <Eigen/Geometry>

TransformationMeta::TransformationMeta(AxisAlignmentRuntime right, AxisAlignmentRuntime forward, AxisAlignmentRuntime up)
	: right(right), forward(forward), up(up)
{
	if (right.axis == forward.axis || forward.axis == up.axis || right.axis == up.axis)
		throw std::exception("The same axis occurs twice!");
}

TransformationMeta::TransformationMeta(const TransformationMeta& other)
	: right(other.right), forward(other.forward), up(other.up)
{
	if (other.right_handed)
		right_handed = std::make_unique<bool>(*other.right_handed);
}

bool TransformationMeta::isRightHanded() const
{
	if (right_handed)
		return *right_handed;

	auto cpy = *this;

	//1. normalize to positive X axis as right axis
	if (right.axis == Axis::X)
	{
		//compare
	}
	else if (forward.axis == Axis::X)
	{
		cpy.rotateLeft();
	}
	else //up.axis == Axis::X
	{
		cpy.rotateRight();
	}
	if (cpy.right.direction == AxisDirection::NEGATIVE)
	{
		cpy.right.direction = AxisDirection::POSITIVE;
		cpy.forward.direction = invert(cpy.forward.direction);
	}
	const bool fw_up_eq = cpy.forward.direction == cpy.up.direction;

	//if (X, Y, Z) then the sign of Y and Z must be equal otherwise they are different
	//e.g. (X, Z, Y)
	right_handed = std::make_unique<bool>((cpy.forward.axis == Axis::Y) ? fw_up_eq : !fw_up_eq);
	return *right_handed;
}

/**
 * convenience method
 */
bool TransformationMeta::isLeftHanded() const
{
	return !isRightHanded();
}

Eigen::Matrix3f TransformationMeta::get_conv_matrix(const TransformationMeta& target) const
{
	Eigen::Matrix3f out = Eigen::Matrix3f::Zero();
	{
		auto [column, row, multiplier] = assignment(right, target.right);
		out(row, column) = multiplier;
	}
	{
		auto [column, row, multiplier] = assignment(forward, target.forward);
		out(row, column) = multiplier;
	}
	{
		auto [column, row, multiplier] = assignment(up, target.up);
		out(row, column) = multiplier;
	}
	return out;
}

Eigen::Matrix4f TransformationMeta::convert_matrix(const TransformationMeta& target, const Eigen::Matrix4f& in) const
{
	const auto ttt = assignments(*this, target);
	return convert(ttt, in);
}

Eigen::Quaternion<float> TransformationMeta::convert_quaternion(const TransformationMeta& target, const Eigen::Quaternion<float>& in) const
{
	Eigen::Quaternion<float> out;

	auto ttt = assignments(*this, target);
	if (isRightHanded() == target.isRightHanded())
		out.w() = in.w();
	else
		out.w() = -in.w();

	auto& coeffs = out.coeffs();
	coeffs[get<1>(ttt[0])] = in.x() * get<2>(ttt[0]);
	coeffs[get<1>(ttt[1])] = in.y() * get<2>(ttt[1]);
	coeffs[get<1>(ttt[2])] = in.z() * get<2>(ttt[2]);

	return out;
}

Eigen::Vector3f TransformationMeta::convert_point(const TransformationMeta& target, const Eigen::Vector3f& in) const
{
	const auto convMatrix = get_conv_matrix(target);
	return convMatrix * in;
}

std::function<Eigen::Matrix4f(const Eigen::Matrix4f&)> TransformationMeta::generate_convert_function(const TransformationMeta& target) const
{
	const auto ttt = assignments(*this, target);
	return [ttt](const Eigen::Matrix4f& in)
		{
			return convert(ttt, in);
		};
}

Eigen::Matrix4f TransformationMeta::convert(const std::array<std::tuple<int8_t, int8_t, float>, 3>& ttt, const Eigen::Matrix4f& in)
{
	Eigen::Matrix4f out = Eigen::Matrix4f::Zero();
	out(3, 3) = 1.f;

	for (size_t y = 0; y < 3; ++y)
	{
		const auto& for_row = ttt[y];

		for (size_t x = 0; x < 3; ++x)
		{
			const auto& for_col = ttt[x];
			out(get<1>(for_row), get<1>(for_col)) = in(y, x) * get<2>(for_row) * get<2>(for_col);
		}
		out(get<1>(for_row), 3) = in(y, 3) * get<2>(for_row);
	}
	return out;
}

void TransformationMeta::rotateLeft()
{
	const auto tmp = right;
	right = forward;
	forward = up;
	up = tmp;
}

void TransformationMeta::rotateRight()
{
	const auto tmp = up;
	up = forward;
	forward = right;
	right = tmp;
}

std::tuple<int8_t, int8_t, float> TransformationMeta::assignment(AxisAlignmentRuntime axis, AxisAlignmentRuntime target_axis)
{
	//column, row, multiplier
	return {
		static_cast<int8_t>(axis.axis),
		static_cast<int8_t>(target_axis.axis),
		static_cast<float>(axis.direction) * static_cast<float>(target_axis.direction)
	};
}

std::array<std::tuple<int8_t, int8_t, float>, 3> TransformationMeta::assignments(const TransformationMeta& origin, const TransformationMeta& target)
{
	std::array<std::tuple<int8_t, int8_t, float>, 3> ttt;
	auto tmp = assignment(origin.right, target.right);
	ttt[get<0>(tmp)] = std::move(tmp);

	tmp = assignment(origin.forward, target.forward);
	ttt[get<0>(tmp)] = std::move(tmp);

	tmp = assignment(origin.up, target.up);
	ttt[get<0>(tmp)] = std::move(tmp);

	return ttt;
}

AxisDirection invert(AxisDirection in)
{
	if (in == AxisDirection::POSITIVE)
		return AxisDirection::NEGATIVE;
	return AxisDirection::POSITIVE;
}