#include "TransformationHelper.h"

#include <Eigen/Geometry>

namespace Transformation
{
	TransformationMeta::TransformationMeta(AxisAlignment right, AxisAlignment forward, AxisAlignment up,
		Ratio scale)
		: scale(scale), m_right(right), m_forward(forward), m_up(up)
	{
		if (right.axis == forward.axis || forward.axis == up.axis || right.axis == up.axis)
			throw std::exception("The same axis occurs twice!");
	}

	TransformationMeta::TransformationMeta(const TransformationMeta& other)
		: scale(other.scale), m_right(other.m_right), m_forward(other.m_forward), m_up(other.m_up)
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
		if (m_right.axis == Axis::X)
		{
			//compare
		}
		else if (m_forward.axis == Axis::X)
		{
			cpy.rotateLeft();
		}
		else //up.axis == Axis::X
		{
			cpy.rotateRight();
		}
		if (cpy.m_right.direction == AxisDirection::NEGATIVE)
		{
			cpy.m_right.direction = AxisDirection::POSITIVE;
			cpy.m_forward.direction = invert(cpy.m_forward.direction);
		}
		const bool fw_up_eq = cpy.m_forward.direction == cpy.m_up.direction;

		//if (X, Y, Z) then the sign of Y and Z must be equal otherwise they are different
		//e.g. (X, Z, Y)
		right_handed = std::make_unique<bool>((cpy.m_forward.axis == Axis::Y) ? fw_up_eq : !fw_up_eq);
		return *right_handed;
	}

	/**
	 * convenience method
	 */
	bool TransformationMeta::isLeftHanded() const
	{
		return !isRightHanded();
	}

	Eigen::Matrix3f TransformationConverter::get_conv_matrix() const
	{
		Eigen::Matrix3f out;

		for (const auto& [column, row, multiplier] : assignments)
		{
			//doesn't matter if we go over x or y for the result
			//only matters for cache performance
			for (int8_t y = 0; y < 3; ++y)
			{
				if (y == row)
					out(row, column) = multiplier * factor;
				else
					out(y, column) = 0.f;
			}
		}

		return out;
	}

	Eigen::Matrix4f TransformationConverter::convert_matrix(const Eigen::Matrix4f& in) const
	{
		return convert(assignments, in, factor);
	}

	Eigen::Quaternion<float> TransformationConverter::convert_quaternion(const Eigen::Quaternion<float>& in) const
	{
		Eigen::Quaternion<float> out;
		
		if (hand_changed)
			out.w() = -in.w();
		else
			out.w() = in.w();

		auto& coeffs = out.coeffs();
		const auto& in_c = in.coeffs();

		for (const auto& [column, row, multiplier] : assignments)
			coeffs[row] = in_c[column] * multiplier;
		
		return out;
	}

	Eigen::Vector3f TransformationConverter::convert_point(const Eigen::Vector3f& in) const
	{
		Eigen::Vector3f out;

		for (const auto& [column, row, multiplier] : assignments)
			out(row, 0) = in(column, 0) * factor * multiplier;

		return out;
	}

	Eigen::Matrix4f TransformationConverter::convert(const SparseAssignments& ttt, const Eigen::Matrix4f& in, float scale)
	{
		Eigen::Matrix4f out;
		for (size_t x = 0; x < 3; ++x)
			out(3, x) = 0.f;
		out(3, 3) = 1.f;

		for (size_t y = 0; y < 3; ++y)
		{
			//column == y
			const auto& [column, out_row, multiplier_y] = ttt[y];

			for (size_t x = 0; x < 3; ++x)
			{
				//exploiting symmetry //row == x
				const auto& [row, out_column, multiplier_x] = ttt[x];
				out(out_row, out_column) = in(y, x) * multiplier_y * multiplier_x;
			}
			out(out_row, 3) = in(y, 3) * multiplier_y;
		}
		for (size_t y = 0; y < 3; ++y)
			out(y, 3) *= scale;
		return out;
	}

	void TransformationMeta::rotateLeft()
	{
		const auto tmp = m_right;
		m_right = m_forward;
		m_forward = m_up;
		m_up = tmp;
	}

	void TransformationMeta::rotateRight()
	{
		const auto tmp = m_up;
		m_up = m_forward;
		m_forward = m_right;
		m_right = tmp;
	}

	Assignment compute_assignment(AxisAlignment axis, AxisAlignment target_axis)
	{
		//column, row, multiplier
		return {
			static_cast<int8_t>(axis.axis),
			static_cast<int8_t>(target_axis.axis),
			static_cast<float>(axis.direction) * static_cast<float>(target_axis.direction)
		};
	}

	SparseAssignments compute_assignments(const TransformationMeta& origin, const TransformationMeta& target)
	{
		SparseAssignments ttt;
		auto tmp = compute_assignment(origin.right(), target.right());
		ttt[std::get<0>(tmp)] = std::move(tmp);

		tmp = compute_assignment(origin.forward(), target.forward());
		ttt[std::get<0>(tmp)] = std::move(tmp);

		tmp = compute_assignment(origin.up(), target.up());
		ttt[std::get<0>(tmp)] = std::move(tmp);

		return ttt;
	}

	AxisDirection invert(AxisDirection in)
	{
		if (in == AxisDirection::POSITIVE)
			return AxisDirection::NEGATIVE;
		return AxisDirection::POSITIVE;
	}

	Ratio::Ratio(std::intmax_t Num, std::intmax_t Denom)
		: Num(Num), Denom(Denom)
	{}

	float Ratio::factor() const
	{
		return static_cast<float>(Num) / static_cast<float>(Denom);
	}

	float Ratio::factor(const Ratio& other) const
	{
		return static_cast<float>(Num * other.Denom) / static_cast<float>(Denom * other.Num);
	}

	TransformationConverter::TransformationConverter(const TransformationMeta& origin, const TransformationMeta& target)
		: factor(origin.scale.factor(target.scale)), assignments(compute_assignments(origin, target)), hand_changed(origin.isRightHanded() != target.isRightHanded())
	{}

	const AxisAlignment& TransformationMeta::right() const
	{
		return m_right;
	}

	const AxisAlignment& TransformationMeta::forward() const
	{
		return m_forward;
	}

	const AxisAlignment& TransformationMeta::up() const
	{
		return m_up;
	}
}