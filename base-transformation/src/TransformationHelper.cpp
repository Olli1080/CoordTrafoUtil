#include "base-transformation/TransformationHelper.h"

namespace Transformation
{
	TransformationMeta::TransformationMeta(AxisAlignment right, AxisAlignment forward, AxisAlignment up,
		Ratio scale)
		: scale(scale), m_right(right), m_forward(forward), m_up(up), m_handedness(Handedness::RIGHT)
	{
		if (right.axis == forward.axis || forward.axis == up.axis || right.axis == up.axis)
			throw std::invalid_argument("The same axis occurs twice!");

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
		bool is_right = (cpy.m_forward.axis == Axis::Y) ? fw_up_eq : !fw_up_eq;
		m_handedness = is_right ? Handedness::RIGHT : Handedness::LEFT;
	}

	Handedness TransformationMeta::handedness() const
	{
		return m_handedness;
	}

	bool TransformationMeta::isRightHanded() const
	{
		return m_handedness == Handedness::RIGHT;
	}

	bool TransformationMeta::isLeftHanded() const
	{
		return m_handedness == Handedness::LEFT;
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
		ttt[tmp.column] = std::move(tmp);

		tmp = compute_assignment(origin.forward(), target.forward());
		ttt[tmp.column] = std::move(tmp);

		tmp = compute_assignment(origin.up(), target.up());
		ttt[tmp.column] = std::move(tmp);

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
	{
		validate();
		simplify();
	}

	void Ratio::validate() const
	{
		if (Denom == 0) throw std::invalid_argument("Denominator cannot be zero");
		if (Num <= 0 || Denom < 0) throw std::invalid_argument("Ratio must be positive");
	}

	void Ratio::simplify()
	{
		auto common = std::gcd(Num, Denom);
		Num /= common;
		Denom /= common;
	}

	float Ratio::factor() const
	{
		return static_cast<float>(Num) / static_cast<float>(Denom);
	}

	float Ratio::factor(const Ratio& other) const
	{
		return static_cast<float>(Num * other.Denom) / static_cast<float>(Denom * other.Num);
	}

	TransformationConverter::TransformationConverter(const TransformationMeta& origin, const TransformationMeta& target)
		: factor(origin.scale.factor(target.scale)), assignments(compute_assignments(origin, target)), hand_changed(origin.handedness() != target.handedness())
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