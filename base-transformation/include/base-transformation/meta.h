#pragma once
#include <string>
#include <base-transformation/types.h>
#include <base-transformation/ratio.h>

namespace Transformation
{
    /**
     * @brief Metadata describing a coordinate system's orientation and scale.
     */
	class TransformationMeta
	{
	public:
        /**
         * @brief Construct metadata by defining the alignment of the Right, Forward, and Up axes.
         * @throws std::invalid_argument if any two axes are the same.
         */
		constexpr TransformationMeta(
			AxisAlignment right,
			AxisAlignment forward,
			AxisAlignment up,
			Ratio scale = {}
		) : scale(scale), m_right(right), m_forward(forward), m_up(up),
            m_handedness(analyze(right, forward, up))
		{}

		bool operator==(const TransformationMeta& other) const = default;

		/** @return The handedness (RIGHT/LEFT) of the system. */
		[[nodiscard]] constexpr Handedness handedness() const { return m_handedness; }
		/** @return True if the system is Right-Handed. */
		[[nodiscard]] constexpr bool isRightHanded() const { return m_handedness == Handedness::RIGHT; }
		/** @return True if the system is Left-Handed. */
		[[nodiscard]] constexpr bool isLeftHanded() const { return m_handedness == Handedness::LEFT; }

		Ratio scale;

		/** @return The alignment of the Right axis. */
		[[nodiscard]] constexpr AxisAlignment right() const { return m_right; }
		/** @return The alignment of the Forward axis. */
		[[nodiscard]] constexpr AxisAlignment forward() const { return m_forward; }
		/** @return The alignment of the Up axis. */
		[[nodiscard]] constexpr AxisAlignment up() const { return m_up; }

	private:
        // Validates the axis triple and derives handedness from the sign of the
        // determinant of the basis [R F U]. Each basis vector has a single non-zero
        // (+/-1) component, so with {r,f,u}.axis a permutation of {0,1,2} the
        // determinant reduces to sign(permutation) * dir_r * dir_f * dir_u.
        // A cyclic permutation of (0,1,2) is even; anything else is odd.
        static constexpr Handedness analyze(AxisAlignment r, AxisAlignment f, AxisAlignment u) {
            if (r.axis == f.axis || f.axis == u.axis || r.axis == u.axis) {
                TRANSFORMATION_THROW(std::invalid_argument, "The same axis occurs twice!");
            }
            const int ri = static_cast<int>(r.axis);
            const int fi = static_cast<int>(f.axis);
            const int parity = ((ri + 1) % 3 == fi) ? 1 : -1;
            const int det = parity
                * static_cast<int>(r.direction)
                * static_cast<int>(f.direction)
                * static_cast<int>(u.direction);
            return det > 0 ? Handedness::RIGHT : Handedness::LEFT;
        }

		AxisAlignment m_right;
		AxisAlignment m_forward;
		AxisAlignment m_up;
		Handedness m_handedness;
	};

    /** @brief Human-readable description, e.g. "TransformationMeta{R=X+, F=Z+, U=Y+, scale=1/1, LEFT}". */
    [[nodiscard]] inline std::string to_string(const TransformationMeta& m) {
        return "TransformationMeta{R=" + to_string(m.right())
             + ", F=" + to_string(m.forward())
             + ", U=" + to_string(m.up())
             + ", scale=" + std::to_string(m.scale.Num) + "/" + std::to_string(m.scale.Denom)
             + ", " + std::string(to_string(m.handedness())) + "}";
    }

    /**
     * @brief Fluent builder for creating TransformationMeta objects.
     */
	class TransformationMetaBuilder {
	public:
		constexpr TransformationMetaBuilder& right(Axis ax, AxisDirection dir) { m_right = { ax, dir }; return *this; }
		constexpr TransformationMetaBuilder& right(AxisAlignment aa) { m_right = aa; return *this; }
		constexpr TransformationMetaBuilder& forward(Axis ax, AxisDirection dir) { m_forward = { ax, dir }; return *this; }
		constexpr TransformationMetaBuilder& forward(AxisAlignment aa) { m_forward = aa; return *this; }
		constexpr TransformationMetaBuilder& up(Axis ax, AxisDirection dir) { m_up = { ax, dir }; return *this; }
		constexpr TransformationMetaBuilder& up(AxisAlignment aa) { m_up = aa; return *this; }
		constexpr TransformationMetaBuilder& scale(Ratio r) { m_scale = r; return *this; }
		constexpr TransformationMetaBuilder& scale(std::intmax_t num, std::intmax_t denom) { m_scale = { num, denom }; return *this; }

		[[nodiscard]] constexpr TransformationMeta build() const {
			return { m_right, m_forward, m_up, m_scale };
		}

	private:
		AxisAlignment m_right{ Axis::X, AxisDirection::POSITIVE };
		AxisAlignment m_forward{ Axis::Y, AxisDirection::POSITIVE };
		AxisAlignment m_up{ Axis::Z, AxisDirection::POSITIVE };
		Ratio m_scale{ 1, 1 };
	};
}
