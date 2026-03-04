#pragma once
#include <cstdint>
#include <stdexcept>

#ifdef TRANSFORMATION_NO_EXCEPTIONS
#include <cassert>
#define TRANSFORMATION_THROW(exc, msg) assert(false && msg)
#else
#define TRANSFORMATION_THROW(exc, msg) throw exc(msg)
#endif

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
            else { TRANSFORMATION_THROW(std::invalid_argument, "Invalid direction in literal (must + or -)"); return {Axis::X, AxisDirection::POSITIVE}; }

            return {ax, dir};
        }
    }
}
