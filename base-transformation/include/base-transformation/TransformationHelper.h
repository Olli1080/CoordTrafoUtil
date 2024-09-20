#pragma once
#include <cstdint>
#include <memory>
#include <ratio>
#include <array>

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
			static_assert(m::size == 3 || m::size == 4);

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
			if constexpr (m::size == 4)
			{
				out(3, 0) = 0.f;
				out(3, 1) = 0.f;
				out(3, 2) = 0.f;
				out(3, 3) = 1.f;

				out(0, 3) = 0.f;
				out(1, 3) = 0.f;
				out(2, 3) = 0.f;
			}

			return out;
		}

		template<matrix_full_access<float> m>
		auto get_conv_matrix() const -> typename m::type
		{
			typename m::type out;
			m proxy(out);
			get_conv_matrix<m>(proxy);

			return out;
		}

		[[nodiscard]] float convert_scale(float scale) const;

		template<quaternion_const_access<float> q_0, quaternion_full_access<float> q_1>
		q_1& convert_quaternion(const q_0 in, q_1& out) const
		{
			if (hand_changed)
				out.set_w(-in.get_w());
			else
				out.set_w(in.get_w());

			for (const auto& [column, row, multiplier] : assignments)
				out(row, in(column) * multiplier);

			return out;
		}

		template<quaternion_full_access<float> q_1, quaternion_const_access<float> q_0>
		auto convert_quaternion(const q_0 in) const -> typename q_1::type
		{
			typename q_1::type out;
			q_1 proxy(out);
			convert_quaternion<q_0, q_1>(in, proxy);

			return out;
		}

		template<matrix_const_access<float> m_0, matrix_full_access<float> m_1>
		m_1& convert_matrix(const m_0 in, m_1& out) const
		{
			convert(assignments, in, out, factor);
			return out;
		}

		template<matrix_full_access<float> m_1, matrix_const_access<float> m_0>
		auto convert_matrix(const m_0 in) const -> typename m_1::type
		{
			typename m_1::type out;
			m_1 proxy(out);
			convert_matrix<m_0, m_1>(in, proxy);

			return out;
		}

		template<vector_const_access<float> v_0, vector_full_access<float> v_1>
		v_1& convert_point(const v_0 in, v_1& out) const
		{
			for (const auto& [column, row, multiplier] : assignments)
				out(row, in(column) * factor * multiplier);

			return out;
		}

		template<vector_full_access<float> v_1, vector_const_access<float> v_0>
		auto convert_point(const v_0 in) const -> typename v_1::type
		{
			typename v_1::type out;
			v_1 proxy(out);
			convert_point<v_0, v_1>(in, proxy);

			return out;
		}

		template<vector_const_access<float> s_0, vector_full_access<float> s_1>
		s_1& convert_size(const s_0 in, s_1& out) const
		{
			for (const auto& [column, row, multiplier] : assignments)
				out(row, in(column) * factor);

			return out;
		}

		template<vector_full_access<float> s_1, vector_const_access<float> s_0>
		auto convert_size(const s_0 in) const -> typename s_1::type
		{
			typename s_1::type out;
			s_1 proxy(out);
			convert_size<s_0, s_1>(in, proxy);

			return out;
		}

	private:

		template<matrix_const_access<float> m_0, matrix_full_access<float> m_1>
		static void convert(const SparseAssignments& ttt, const m_0& in, m_1& out, float scale)
		{
			static_assert(m_0::size == 4 && m_1::size == 4);

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
				out(out_row, 3) = in(y, 3) * multiplier_y * scale;
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