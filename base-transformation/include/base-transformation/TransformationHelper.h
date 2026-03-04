#pragma once
#include <cstdint>
#include <memory>
#include <ratio>
#include <array>
#include <tuple>

#include "concepts.h"

namespace Transformation
{
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

	//column, row, multiplier
	typedef std::tuple<int8_t, int8_t, float> Assignment;
	typedef std::array<Assignment, 3> SparseAssignments;

	class TransformationMeta;
	static Assignment compute_assignment(AxisAlignment axis, AxisAlignment target_axis);
	static SparseAssignments compute_assignments(const TransformationMeta& origin, const TransformationMeta& target);


	class TransformationConverter
	{
	public:

		TransformationConverter(const TransformationMeta& origin, const TransformationMeta& target);

		template<matrix_full_access<float> m>
		m& get_conv_matrix(m& out) const
		{
			constexpr size_t size = MatrixTraits<m, float>::size;
			static_assert(size == 3 || size == 4);

			for (const auto& [column, row, multiplier] : assignments)
			{
				for (int8_t y = 0; y < 3; ++y)
				{
					if (y == row)
						MatrixTraits<m, float>::set(out, row, column, multiplier * factor);
					else
						MatrixTraits<m, float>::set(out, y, column, 0.f);
				}
			}
			if constexpr (size == 4)
			{
				MatrixTraits<m, float>::set(out, 3, 0, 0.f);
				MatrixTraits<m, float>::set(out, 3, 1, 0.f);
				MatrixTraits<m, float>::set(out, 3, 2, 0.f);
				MatrixTraits<m, float>::set(out, 3, 3, 1.f);

				MatrixTraits<m, float>::set(out, 0, 3, 0.f);
				MatrixTraits<m, float>::set(out, 1, 3, 0.f);
				MatrixTraits<m, float>::set(out, 2, 3, 0.f);
			}

			return out;
		}

		[[nodiscard]] float convert_scale(float scale) const;

		template<quaternion_const_access<float> q_0, quaternion_full_access<float> q_1>
		q_1& convert_quaternion(const q_0& in, q_1& out) const
		{
			if (hand_changed)
				QuaternionTraits<q_1, float>::set_w(out, -QuaternionTraits<q_0, float>::get_w(in));
			else
				QuaternionTraits<q_1, float>::set_w(out, QuaternionTraits<q_0, float>::get_w(in));

			for (const auto& [column, row, multiplier] : assignments)
				QuaternionTraits<q_1, float>::set_idx(out, row, QuaternionTraits<q_0, float>::get_idx(in, column) * multiplier);

			return out;
		}

		template<matrix_const_access<float> m_0, matrix_full_access<float> m_1>
		m_1& convert_matrix(const m_0& in, m_1& out) const
		{
			convert(assignments, in, out, factor);
			return out;
		}

		template<vector_const_access<float> v_0, vector_full_access<float> v_1>
		v_1& convert_point(const v_0& in, v_1& out) const
		{
			for (const auto& [column, row, multiplier] : assignments)
				VectorTraits<v_1, float>::set_idx(out, row, VectorTraits<v_0, float>::get_idx(in, column) * factor * multiplier);

			return out;
		}

		template<vector_const_access<float> s_0, vector_full_access<float> s_1>
		s_1& convert_size(const s_0& in, s_1& out) const
		{
			for (const auto& [column, row, multiplier] : assignments)
				VectorTraits<s_1, float>::set_idx(out, row, VectorTraits<s_0, float>::get_idx(in, column) * factor);

			return out;
		}

	private:

		template<matrix_const_access<float> m_0, matrix_full_access<float> m_1>
		static void convert(const SparseAssignments& ttt, const m_0& in, m_1& out, float scale)
		{
			static_assert(MatrixTraits<m_0, float>::size == 4 && MatrixTraits<m_1, float>::size == 4);

			for (size_t x = 0; x < 3; ++x)
				MatrixTraits<m_1, float>::set(out, 3, x, 0.f);
			MatrixTraits<m_1, float>::set(out, 3, 3, 1.f);

			for (size_t y = 0; y < 3; ++y)
			{
				const auto& [column, out_row, multiplier_y] = ttt[y];

				for (size_t x = 0; x < 3; ++x)
				{
					const auto& [row, out_column, multiplier_x] = ttt[x];
					float val = MatrixTraits<m_0, float>::get(in, y, x);
					MatrixTraits<m_1, float>::set(out, out_row, out_column, val * multiplier_y * multiplier_x);
				}
				float trans_val = MatrixTraits<m_0, float>::get(in, y, 3);
				MatrixTraits<m_1, float>::set(out, out_row, 3, trans_val * multiplier_y * scale);
			}
		}

		float factor;
		SparseAssignments assignments;
		bool hand_changed;
	};

	class TransformationMeta
	{
	public:

		TransformationMeta(
			AxisAlignment right,
			AxisAlignment forward,
			AxisAlignment up,
			Ratio scale = { 1, 1 }
		);

		TransformationMeta(const TransformationMeta& other);

		[[nodiscard]] bool isRightHanded() const;
		[[nodiscard]] bool isLeftHanded() const;

		Ratio scale;

		const AxisAlignment& right() const;
		const AxisAlignment& forward() const;
		const AxisAlignment& up() const;

	private:

		void rotateLeft();
		void rotateRight();

		AxisAlignment m_right;
		AxisAlignment m_forward;
		AxisAlignment m_up;

		mutable std::unique_ptr<bool> right_handed;
	};
}