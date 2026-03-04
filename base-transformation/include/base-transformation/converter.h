#pragma once
#include <span>
#include <array>
#include <algorithm>
#include <base-transformation/concepts.h>
#include <base-transformation/meta.h>

namespace Transformation
{
    /**
     * @brief Performs the actual data transformation between two coordinate systems.
     * @tparam T The scalar type (float, double, etc.) used for pre-computed factors.
     */
    template<typename T = float>
	class TransformationConverter
	{
	public:
        /** @brief Internal mapping structure for a single axis. */
        struct Assignment {
            int8_t origin_axis;
            int8_t target_axis;
            T multiplier;
        };
        /** @brief Sparse representation of the 3x3 transformation logic. */
        typedef std::array<Assignment, DIM_3D> SparseAssignments;

        /**
         * @brief Construct a converter between an origin and a target coordinate system.
         * @param origin Source coordinate system metadata.
         * @param target Destination coordinate system metadata.
         */
		TransformationConverter(const TransformationMeta& origin, const TransformationMeta& target)
            : factor(origin.scale.template factor<T>(target.scale)), 
              assignments(compute_assignments(origin, target)), 
              hand_changed(origin.handedness() != target.handedness())
        {}

        /**
         * @brief Fills a transformation matrix representing the conversion.
         */
		template<matrix_full_access<T> m>
		m& get_conv_matrix(m& out) const
		{
			constexpr size_t size = MatrixTraits<m, T>::size;
			static_assert(size == DIM_3D || size == DIM_4D);

			for (const auto& asgn : assignments)
			{
				for (int8_t y = 0; y < static_cast<int8_t>(DIM_3D); ++y)
				{
					if (y == asgn.target_axis)
						MatrixTraits<m, T>::set(out, asgn.target_axis, asgn.origin_axis, asgn.multiplier * factor);
					else
						MatrixTraits<m, T>::set(out, y, asgn.origin_axis, static_cast<T>(0));
				}
			}
			if constexpr (size == DIM_4D)
			{
				MatrixTraits<m, T>::set(out, 3, 0, static_cast<T>(0));
				MatrixTraits<m, T>::set(out, 3, 1, static_cast<T>(0));
				MatrixTraits<m, T>::set(out, 3, 2, static_cast<T>(0));
				MatrixTraits<m, T>::set(out, 3, 3, static_cast<T>(1));

				MatrixTraits<m, T>::set(out, 0, 3, static_cast<T>(0));
				MatrixTraits<m, T>::set(out, 1, 3, static_cast<T>(0));
				MatrixTraits<m, T>::set(out, 2, 3, static_cast<T>(0));
			}
			return out;
		}

        /**
         * @brief Returns a transformation matrix by value.
         */
        template<matrix_full_access<T> m>
        auto get_conv_matrix() const -> typename MatrixTraits<m, T>::type {
            typename MatrixTraits<m, T>::type out;
            get_conv_matrix(out);
            return out;
        }

		[[nodiscard]] T convert_scale(T scale) const { return factor * scale; }

		template<quaternion_const_access<T> q_in, quaternion_full_access<T> q_out>
		q_out& convert_quaternion(const q_in& in, q_out& out) const
		{
			QuaternionTraits<q_out, T>::set_w(out, hand_changed ? -QuaternionTraits<q_in, T>::get_w(in) : QuaternionTraits<q_in, T>::get_w(in));

			for (const auto& asgn : assignments)
				QuaternionTraits<q_out, T>::set_idx(out, asgn.target_axis, QuaternionTraits<q_in, T>::get_idx(in, asgn.origin_axis) * asgn.multiplier);

			return out;
		}

        template<quaternion_full_access<T> q_out, quaternion_const_access<T> q_in>
        auto convert_quaternion(const q_in& in) const -> typename QuaternionTraits<q_out, T>::type {
            typename QuaternionTraits<q_out, T>::type out;
            convert_quaternion(in, out);
            return out;
        }

		template<matrix_const_access<T> m_in, matrix_full_access<T> m_out>
		m_out& convert_matrix(const m_in& in, m_out& out) const
		{
			convert(assignments, in, out, factor);
			return out;
		}

        template<matrix_full_access<T> m_out, matrix_const_access<T> m_in>
        auto convert_matrix(const m_in& in) const -> typename MatrixTraits<m_out, T>::type {
            typename MatrixTraits<m_out, T>::type out;
            convert_matrix(in, out);
            return out;
        }

		template<vector_const_access<T> v_in, vector_full_access<T> v_out>
		v_out& convert_point(const v_in& in, v_out& out) const
		{
			for (const auto& asgn : assignments)
				VectorTraits<v_out, T>::set_idx(out, asgn.target_axis, VectorTraits<v_in, T>::get_idx(in, asgn.origin_axis) * factor * asgn.multiplier);
			return out;
		}

        template<vector_full_access<T> v_out, vector_const_access<T> v_in>
        auto convert_point(const v_in& in) const -> typename VectorTraits<v_out, T>::type {
            typename VectorTraits<v_out, T>::type out;
            convert_point(in, out);
            return out;
        }

		template<vector_const_access<T> s_in, vector_full_access<T> s_out>
		s_out& convert_size(const s_in& in, s_out& out) const
		{
			for (const auto& asgn : assignments)
				VectorTraits<s_out, T>::set_idx(out, asgn.target_axis, VectorTraits<s_in, T>::get_idx(in, asgn.origin_axis) * factor);
			return out;
		}

        template<vector_full_access<T> s_out, vector_const_access<T> s_in>
        auto convert_size(const s_in& in) const -> typename VectorTraits<s_out, T>::type {
            typename VectorTraits<s_out, T>::type out;
            convert_size(in, out);
            return out;
        }

        /**
         * @brief Batch conversion of points.
         */
        template<vector_const_access<T> v_in, vector_full_access<T> v_out>
        void convert_points(std::span<const v_in> in, std::span<v_out> out) const {
            const size_t count = std::min(in.size(), out.size());
            const T f = factor;
            for (size_t i = 0; i < count; ++i) {
                for (const auto& asgn : assignments) {
                    T val = VectorTraits<v_in, T>::get_idx(in[i], asgn.origin_axis);
                    VectorTraits<v_out, T>::set_idx(out[i], asgn.target_axis, val * f * asgn.multiplier);
                }
            }
        }

        /**
         * @brief Batch conversion of sizes.
         */
        template<vector_const_access<T> v_in, vector_full_access<T> v_out>
        void convert_sizes(std::span<const v_in> in, std::span<v_out> out) const {
            const size_t count = std::min(in.size(), out.size());
            const T f = factor;
            for (size_t i = 0; i < count; ++i) {
                for (const auto& asgn : assignments) {
                    T val = VectorTraits<v_in, T>::get_idx(in[i], asgn.origin_axis);
                    VectorTraits<v_out, T>::set_idx(out[i], asgn.target_axis, val * f);
                }
            }
        }

	private:
		template<matrix_const_access<T> m_in, matrix_full_access<T> m_out>
		static void convert(const SparseAssignments& ttt, const m_in& in, m_out& out, T scale)
		{
			static_assert(MatrixTraits<m_in, T>::size == DIM_4D && MatrixTraits<m_out, T>::size == DIM_4D);

			for (size_t x = 0; x < DIM_3D; ++x)
				MatrixTraits<m_out, T>::set(out, 3, x, static_cast<T>(0));
			MatrixTraits<m_out, T>::set(out, 3, 3, static_cast<T>(1));

			for (size_t y = 0; y < DIM_3D; ++y)
			{
				const auto& asgn_y = ttt[y];
				for (size_t x = 0; x < DIM_3D; ++x)
				{
					const auto& asgn_x = ttt[x];
					T val = MatrixTraits<m_in, T>::get(in, y, x);
					MatrixTraits<m_out, T>::set(out, asgn_y.target_axis, asgn_x.target_axis, val * asgn_y.multiplier * asgn_x.multiplier);
				}
				T trans_val = MatrixTraits<m_in, T>::get(in, y, 3);
				MatrixTraits<m_out, T>::set(out, asgn_y.target_axis, 3, trans_val * asgn_y.multiplier * scale);
			}
		}

        static constexpr Assignment compute_assignment(AxisAlignment axis, AxisAlignment target_axis) {
            return {
                static_cast<int8_t>(axis.axis),
                static_cast<int8_t>(target_axis.axis),
                static_cast<T>(axis.direction) * static_cast<T>(target_axis.direction)
            };
        }

        static constexpr SparseAssignments compute_assignments(const TransformationMeta& origin, const TransformationMeta& target) {
            SparseAssignments ttt;
            auto r = compute_assignment(origin.right(), target.right());
            ttt[r.origin_axis] = r;
            auto f = compute_assignment(origin.forward(), target.forward());
            ttt[f.origin_axis] = f;
            auto u = compute_assignment(origin.up(), target.up());
            ttt[u.origin_axis] = u;
            return ttt;
        }

		T factor;
		SparseAssignments assignments;
		bool hand_changed;
	};
}
