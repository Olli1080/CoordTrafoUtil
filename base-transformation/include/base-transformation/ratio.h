#pragma once
#include <cstdint>
#include <ratio>
#include <numeric>
#include <base-transformation/types.h>

namespace Transformation
{
    /**
     * @brief Represents a scale factor as a rational number (Num/Denom).
     * 
     * Used to handle units (e.g., 1/1 for meters, 1/100 for centimeters).
     * Ratios are automatically simplified and validated during construction.
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
}
