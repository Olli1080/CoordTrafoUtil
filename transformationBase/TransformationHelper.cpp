#include "TransformationHelper.h"

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

		TransformationMeta cpy = *this;

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

	float TransformationConverter::convert_scale(float scale) const
	{
		return factor * scale;
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