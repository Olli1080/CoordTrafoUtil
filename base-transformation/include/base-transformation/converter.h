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
        /**
         * @brief Internal mapping structure for a single axis.
         *
         * Represents how an axis from the origin system maps to an axis in the target system.
         * For example, if Origin Right (+X) maps to Target Up (+Y), the assignment would be:
         * origin_axis: 0 (X), target_axis: 1 (Y), multiplier: 1.0.
         */
        struct Assignment {
            int8_t origin_axis = 0; /**< Index of the axis in the source system (0=X, 1=Y, 2=Z). */
            int8_t target_axis = 0; /**< Index of the axis in the destination system (0=X, 1=Y, 2=Z). */
            T multiplier = 0;       /**< Sign multiplier (+1 or -1) reflecting direction alignment. */
        };
        /**
         * @brief Sparse representation of the 3x3 transformation logic.
         *
         * Contains three assignments, one for each basis vector (Right, Forward, Up).
         * This allows the library to perform transformations using sparse assignments
         * rather than dense matrix multiplications, providing maximum efficiency.
         */
        typedef std::array<Assignment, DIM_3D> SparseAssignments;

        /**
         * @brief Construct a converter between an origin and a target coordinate system.
         * @param origin Source coordinate system metadata.
         * @param target Destination coordinate system metadata.
         *
         * All inputs are constant-expression friendly, so a converter can be built at
         * compile time (`static constexpr TransformationConverter c{a, b};`) as well as
         * at runtime (e.g. from deserialized coordinate-system descriptions).
         */
		constexpr TransformationConverter(const TransformationMeta& origin, const TransformationMeta& target)
            : m_origin(origin),
              m_target(target),
              factor(origin.scale.template factor<T>(target.scale)),
              assignments(compute_assignments(origin, target)),
              hand_changed(origin.handedness() != target.handedness())
        {}

        /** @return The origin coordinate system this converter maps from. */
        [[nodiscard]] constexpr const TransformationMeta& origin() const { return m_origin; }
        /** @return The target coordinate system this converter maps to. */
        [[nodiscard]] constexpr const TransformationMeta& target() const { return m_target; }

        /** @return A converter performing the opposite conversion (target -> origin). */
        [[nodiscard]] constexpr TransformationConverter inverse() const {
            return { m_target, m_origin };
        }

        /**
         * @brief Compose with another converter: `a.then(b)` converts a.origin -> b.target.
         * @throws std::invalid_argument if this->target() does not match next.origin().
         */
        [[nodiscard]] constexpr TransformationConverter then(const TransformationConverter& next) const {
            if (!(m_target == next.m_origin)) {
                TRANSFORMATION_THROW(std::invalid_argument,
                    "TransformationConverter::then: intermediate coordinate systems do not match");
            }
            return { m_origin, next.m_target };
        }

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

		[[nodiscard]] constexpr T convert_scale(T scale) const { return factor * scale; }

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

        /**
         * @brief Converts a matrix between coordinate systems.
         *
         * A 4x4 matrix is treated as an affine transform: the 3x3 linear block is
         * re-based and the translation column is re-based and scaled. A 3x3 matrix is
         * treated as a pure linear/rotation transform (no scaling is applied, since a
         * rotation is scale-invariant).
         */
		template<matrix_const_access<T> m_in, matrix_full_access<T> m_out>
		m_out& convert_matrix(const m_in& in, m_out& out) const
		{
			constexpr size_t si = MatrixTraits<m_in, T>::size;
			constexpr size_t so = MatrixTraits<m_out, T>::size;
			static_assert(si == so, "convert_matrix: input and output matrix sizes must match");
			static_assert(si == DIM_3D || si == DIM_4D, "convert_matrix: only 3x3 and 4x4 matrices are supported");

			if constexpr (si == DIM_4D)
				convert_affine(assignments, in, out, factor);
			else
				convert_linear(assignments, in, out);
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
			// Sizes/extents are unsigned magnitudes: re-base the axes and apply the
			// scale factor, but never the direction sign.
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
         * @brief Batch conversion of points. Processes min(in.size(), out.size()) elements.
         */
        template<vector_const_access<T> v_in, vector_full_access<T> v_out>
        void convert_points(std::span<const v_in> in, std::span<v_out> out) const {
            const size_t count = std::min(in.size(), out.size());
            for (size_t i = 0; i < count; ++i)
                convert_point(in[i], out[i]);
        }

        /**
         * @brief Batch conversion of sizes. Processes min(in.size(), out.size()) elements.
         */
        template<vector_const_access<T> v_in, vector_full_access<T> v_out>
        void convert_sizes(std::span<const v_in> in, std::span<v_out> out) const {
            const size_t count = std::min(in.size(), out.size());
            for (size_t i = 0; i < count; ++i)
                convert_size(in[i], out[i]);
        }

	private:
		template<matrix_const_access<T> m_in, matrix_full_access<T> m_out>
		static void convert_affine(const SparseAssignments& ttt, const m_in& in, m_out& out, T scale)
		{
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

		template<matrix_const_access<T> m_in, matrix_full_access<T> m_out>
		static void convert_linear(const SparseAssignments& ttt, const m_in& in, m_out& out)
		{
			for (size_t y = 0; y < DIM_3D; ++y)
			{
				const auto& asgn_y = ttt[y];
				for (size_t x = 0; x < DIM_3D; ++x)
				{
					const auto& asgn_x = ttt[x];
					T val = MatrixTraits<m_in, T>::get(in, y, x);
					MatrixTraits<m_out, T>::set(out, asgn_y.target_axis, asgn_x.target_axis, val * asgn_y.multiplier * asgn_x.multiplier);
				}
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
            SparseAssignments ttt{};
            const auto r = compute_assignment(origin.right(), target.right());
            ttt[r.origin_axis] = r;
            const auto f = compute_assignment(origin.forward(), target.forward());
            ttt[f.origin_axis] = f;
            const auto u = compute_assignment(origin.up(), target.up());
            ttt[u.origin_axis] = u;
            return ttt;
        }

		TransformationMeta m_origin;
		TransformationMeta m_target;
		T factor;
		SparseAssignments assignments;
		bool hand_changed;
	};
}
