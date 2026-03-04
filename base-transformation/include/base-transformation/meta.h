#pragma once
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
}
