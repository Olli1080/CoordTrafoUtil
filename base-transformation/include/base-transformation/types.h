#pragma once
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

#ifdef TRANSFORMATION_NO_EXCEPTIONS
#include <cstdio>
#include <cstdlib>

namespace Transformation::detail
{
    /**
     * @brief Terminating error handler used when the library is built without exceptions.
     *
     * Reports @p msg to stderr and aborts. Marked [[noreturn]] so callers do not fall
     * through to a bogus "recovery" value. If this is ever reached during constant
     * evaluation the build fails, which is the desired behaviour for bad literals.
     */
    [[noreturn]] inline void fail(const char* msg) noexcept
    {
        std::fprintf(stderr, "CoordTrafoUtil fatal: %s\n", msg);
        std::abort();
    }
}

#define TRANSFORMATION_THROW(exc, msg) ::Transformation::detail::fail(msg)
#else
#define TRANSFORMATION_THROW(exc, msg) throw exc(msg)
#endif

namespace Transformation
{
    /** @brief Number of dimensions in a standard 3D vector. */
    inline constexpr size_t DIM_3D = 3;
    /** @brief Number of dimensions in a 4x4 transformation matrix. */
    inline constexpr size_t DIM_4D = 4;

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
        inline constexpr AxisAlignment X_pos = {Axis::X, AxisDirection::POSITIVE};
        inline constexpr AxisAlignment X_neg = {Axis::X, AxisDirection::NEGATIVE};
        inline constexpr AxisAlignment Y_pos = {Axis::Y, AxisDirection::POSITIVE};
        inline constexpr AxisAlignment Y_neg = {Axis::Y, AxisDirection::NEGATIVE};
        inline constexpr AxisAlignment Z_pos = {Axis::Z, AxisDirection::POSITIVE};
        inline constexpr AxisAlignment Z_neg = {Axis::Z, AxisDirection::NEGATIVE};
    }

    /** @brief Short name of an axis ("X", "Y", "Z"). */
    [[nodiscard]] constexpr std::string_view to_string(Axis a) {
        switch (a) {
            case Axis::X: return "X";
            case Axis::Y: return "Y";
            case Axis::Z: return "Z";
        }
        return "?";
    }

    /** @brief Sign of a direction ("+", "-"). */
    [[nodiscard]] constexpr std::string_view to_string(AxisDirection d) {
        return d == AxisDirection::POSITIVE ? "+" : "-";
    }

    /** @brief Handedness name ("RIGHT", "LEFT"). */
    [[nodiscard]] constexpr std::string_view to_string(Handedness h) {
        return h == Handedness::RIGHT ? "RIGHT" : "LEFT";
    }

    /** @brief Axis alignment in "X+" form. */
    [[nodiscard]] inline std::string to_string(AxisAlignment aa) {
        return std::string(to_string(aa.axis)) + std::string(to_string(aa.direction));
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
        constexpr AxisAlignment operator""_a(const char* str, std::size_t len) {
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
