#pragma once
#include <cstdint>
#include <ratio>
#include <array>
#include <tuple>
#include <numeric>
#include <stdexcept>
#include <span>
#include <algorithm>
#include <string_view>

#ifdef TRANSFORMATION_NO_EXCEPTIONS
#include <cassert>
#define TRANSFORMATION_THROW(exc, msg) assert(false && msg)
#else
#define TRANSFORMATION_THROW(exc, msg) throw exc(msg)
#endif

#include <base-transformation/concepts.h>

namespace Transformation
{
    /** @brief Number of dimensions in a standard 3D vector. */
    static constexpr size_t DIM_3D = 3;
    /** @brief Number of dimensions in a 4x4 transformation matrix. */
    static constexpr size_t DIM_4D = 4;

    /**
     * @brief Identifies a principal axis in a 3D coordinate system.
     */
	enum class Axis : int8_t
	{
		X = 0,
		Y = 1,
		Z = 2
	};

    /**
     * @brief Identifies the direction of an axis.
     */
	enum class AxisDirection : int8_t
	{
		POSITIVE = +1,
		NEGATIVE = -1
	};

    /**
     * @brief Inverts an axis direction.
     * @param in The direction to invert.
     * @return The inverted direction.
     */
	[[nodiscard]] constexpr AxisDirection invert(AxisDirection in) {
		return (in == AxisDirection::POSITIVE) ? AxisDirection::NEGATIVE : AxisDirection::POSITIVE;
	}

    /**
     * @brief Identifies the handedness of a coordinate system.
     */
	enum class Handedness
	{
		RIGHT,
		LEFT
	};

    /**
     * @brief Combines an axis and its direction.
     */
	struct AxisAlignment
	{
		Axis axis;
		AxisDirection direction;

		bool operator==(const AxisAlignment& other) const = default;
	};

    /**
     * @brief Constants for standard axis alignments.
     */
    namespace AxisAlignments {
        static constexpr AxisAlignment X_pos = {Axis::X, AxisDirection::POSITIVE};
        static constexpr AxisAlignment X_neg = {Axis::X, AxisDirection::NEGATIVE};
        static constexpr AxisAlignment Y_pos = {Axis::Y, AxisDirection::POSITIVE};
        static constexpr AxisAlignment Y_neg = {Axis::Y, AxisDirection::NEGATIVE};
        static constexpr AxisAlignment Z_pos = {Axis::Z, AxisDirection::POSITIVE};
        static constexpr AxisAlignment Z_neg = {Axis::Z, AxisDirection::NEGATIVE};
    }

    /**
     * @brief Inline namespace for axis alignment literals.
     */
    inline namespace Literals {
        /** 
         * @brief Literal for axis alignment (e.g. "X+"_a, "Z-"_a).
         * @param str The literal string (must be 2 chars: [X|Y|Z][+|-]).
         * @param len The length of the string.
         */
        constexpr AxisAlignment operator"" _a(const char* str, std::size_t len) {
            if (len != 2) { TRANSFORMATION_THROW(std::invalid_argument, "Axis literal must be 2 chars, e.g., 'X+'"); return {Axis::X, AxisDirection::POSITIVE}; }
            
            Axis ax;
            if (str[0] == 'X' || str[0] == 'x') ax = Axis::X;
            else if (str[0] == 'Y' || str[0] == 'y') ax = Axis::Y;
            else if (str[0] == 'Z' || str[0] == 'z') ax = Axis::Z;
            else { TRANSFORMATION_THROW(std::invalid_argument, "Invalid axis in literal (must be X, Y, or Z)"); return {Axis::X, AxisDirection::POSITIVE}; }

            AxisDirection dir;
            if (str[1] == '+') dir = AxisDirection::POSITIVE;
            else if (str[1] == '-') dir = AxisDirection::NEGATIVE;
            else { TRANSFORMATION_THROW(std::invalid_argument, "Invalid direction in literal (must be + or -)"); return {Axis::X, AxisDirection::POSITIVE}; }

            return {ax, dir};
        }
    }

    /**
     * @brief Represents a scale factor as a rational number (Num/Denom).
     */
	struct Ratio
	{
		std::intmax_t Num;
		std::intmax_t Denom;

        /**
         * @brief Construct a ratio from numerator and denominator.
         */
		constexpr Ratio(std::intmax_t Num, std::intmax_t Denom) : Num(Num), Denom(Denom) {
			validate();
			simplify();
		}

        /** @brief Default constructor for 1/1 ratio. */
		template<std::intmax_t N, std::intmax_t D>
		inline constexpr Ratio() : Num(N), Denom(D) {
			validate();
			simplify();
		}

        /** @brief Construct from a std::ratio. */
		template<std::intmax_t N, std::intmax_t D>
		inline constexpr Ratio(std::ratio<N, D>) : Num(N), Denom(D) {
			validate();
			simplify();
		}

        /** @brief Calculate the scalar factor of this ratio. */
		template<typename T = float>
		[[nodiscard]] constexpr T factor() const {
			return static_cast<T>(Num) / static_cast<T>(Denom);
		}

        /** @brief Calculate the conversion factor from this ratio to another. */
		template<typename T = float>
		[[nodiscard]] constexpr T factor(const Ratio& other) const {
			return static_cast<T>(Num * other.Denom) / static_cast<T>(Denom * other.Num);
		}

		bool operator==(const Ratio& other) const = default;

	private:
		constexpr void validate() const {
			if (Denom == 0) { TRANSFORMATION_THROW(std::invalid_argument, "Denominator cannot be zero"); }
			if (Num <= 0 || (Num > 0 && Denom < 0)) { TRANSFORMATION_THROW(std::invalid_argument, "Ratio must be positive"); }
		}
		constexpr void simplify() {
			auto common = std::gcd(Num, Denom);
			Num /= common;
			Denom /= common;
		}
	};

    /**
     * @brief Metadata describing a coordinate system's orientation and scale.
     */
	class TransformationMeta
	{
	public:
        /**
         * @brief Construct metadata by defining the alignment of the Right, Forward, and Up axes.
         */
		constexpr TransformationMeta(
			AxisAlignment right,
			AxisAlignment forward,
			AxisAlignment up,
			Ratio scale = { 1, 1 }
		) : scale(scale), m_right(right), m_forward(forward), m_up(up), 
            m_handedness(calculate_handedness(right, forward, up))
		{
			if (right.axis == forward.axis || forward.axis == up.axis || right.axis == up.axis) {
				TRANSFORMATION_THROW(std::invalid_argument, "The same axis occurs twice!");
            }
		}

		bool operator==(const TransformationMeta& other) const = default;

		/** @return The handedness (RIGHT/LEFT) of the system. */
		[[nodiscard]] constexpr Handedness handedness() const { return m_handedness; }
		/** @return True if the system is Right-Handed. */
		[[nodiscard]] constexpr bool isRightHanded() const { return m_handedness == Handedness::RIGHT; }
		/** @return True if the system is Left-Handed. */
		[[nodiscard]] constexpr bool isLeftHanded() const { return m_handedness == Handedness::LEFT; }

		Ratio scale;

		/** @return The alignment of the Right axis. */
		const AxisAlignment& right() const { return m_right; }
		/** @return The alignment of the Forward axis. */
		const AxisAlignment& forward() const { return m_forward; }
		/** @return The alignment of the Up axis. */
		const AxisAlignment& up() const { return m_up; }

	private:
        static constexpr void rotateLeft(AxisAlignment& r, AxisAlignment& f, AxisAlignment& u) {
            const auto tmp = r; r = f; f = u; u = tmp;
        }

        static constexpr void rotateRight(AxisAlignment& r, AxisAlignment& f, AxisAlignment& u) {
            const auto tmp = u; u = f; f = r; r = tmp;
        }

        static constexpr Handedness calculate_handedness(AxisAlignment r, AxisAlignment f, AxisAlignment u) {
            if (f.axis == Axis::X) {
                rotateLeft(r, f, u);
            } else if (u.axis == Axis::X) {
                rotateRight(r, f, u);
            }
            if (r.direction == AxisDirection::NEGATIVE) {
                r.direction = AxisDirection::POSITIVE;
                f.direction = invert(f.direction);
            }
            const bool fw_up_eq = f.direction == u.direction;
            bool is_right = (f.axis == Axis::Y) ? fw_up_eq : !fw_up_eq;
            return is_right ? Handedness::RIGHT : Handedness::LEFT;
        }

		AxisAlignment m_right;
		AxisAlignment m_forward;
		AxisAlignment m_up;
		Handedness m_handedness;
	};

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

    /**
     * @brief Fluent builder for creating TransformationMeta objects.
     */
	class TransformationMetaBuilder {
	public:
		TransformationMetaBuilder& right(Axis ax, AxisDirection dir) { m_right = { ax, dir }; return *this; }
		TransformationMetaBuilder& right(AxisAlignment aa) { m_right = aa; return *this; }
		TransformationMetaBuilder& forward(Axis ax, AxisDirection dir) { m_forward = { ax, dir }; return *this; }
		TransformationMetaBuilder& forward(AxisAlignment aa) { m_forward = aa; return *this; }
		TransformationMetaBuilder& up(Axis ax, AxisDirection dir) { m_up = { ax, dir }; return *this; }
		TransformationMetaBuilder& up(AxisAlignment aa) { m_up = aa; return *this; }
		TransformationMetaBuilder& scale(Ratio r) { m_scale = r; return *this; }
		TransformationMetaBuilder& scale(std::intmax_t num, std::intmax_t denom) { m_scale = { num, denom }; return *this; }

		TransformationMeta build() const {
			return { m_right, m_forward, m_up, m_scale };
		}

	private:
		AxisAlignment m_right{ Axis::X, AxisDirection::POSITIVE };
		AxisAlignment m_forward{ Axis::Y, AxisDirection::POSITIVE };
		AxisAlignment m_up{ Axis::Z, AxisDirection::POSITIVE };
		Ratio m_scale{ 1, 1 };
	};

    /**
     * @brief Presets for common coordinate systems.
     */
    namespace Presets {
        /** @brief Unity: Left-handed, X: Right, Y: Up, Z: Forward. (Meters) */
        static constexpr TransformationMeta Unity() {
            return { {Axis::X, AxisDirection::POSITIVE}, {Axis::Z, AxisDirection::POSITIVE}, {Axis::Y, AxisDirection::POSITIVE} };
        }
        /** @brief Unreal: Left-handed, X: Forward, Y: Right, Z: Up. (Centimeters) */
        static constexpr TransformationMeta Unreal() {
            return { {Axis::Y, AxisDirection::POSITIVE}, {Axis::X, AxisDirection::POSITIVE}, {Axis::Z, AxisDirection::POSITIVE}, {1, 100} };
        }
        /** @brief OpenGL/Right-Handed: Right: X+, Forward: -Z, Up: Y+. */
        static constexpr TransformationMeta OpenGL() {
            return { {Axis::X, AxisDirection::POSITIVE}, {Axis::Z, AxisDirection::NEGATIVE}, {Axis::Y, AxisDirection::POSITIVE} };
        }
        /** @brief ROS: Right-handed, X: Forward, Y: Left, Z: Up. */
        static constexpr TransformationMeta ROS() {
            return { {Axis::Y, AxisDirection::NEGATIVE}, {Axis::X, AxisDirection::POSITIVE}, {Axis::Z, AxisDirection::POSITIVE} };
        }
    }
}
