#pragma once
#include <base-transformation/meta.h>

namespace Transformation
{
    /**
     * @brief Presets for common coordinate systems.
     */
    namespace Presets {
        /** @brief Unity: Left-handed, X: Right, Y: Up, Z: Forward. (Meters) */
        inline constexpr TransformationMeta Unity() {
            return { {Axis::X, AxisDirection::POSITIVE}, {Axis::Z, AxisDirection::POSITIVE}, {Axis::Y, AxisDirection::POSITIVE} };
        }
        /** @brief Unreal: Left-handed, X: Forward, Y: Right, Z: Up. (Centimeters) */
        inline constexpr TransformationMeta Unreal() {
            return { {Axis::Y, AxisDirection::POSITIVE}, {Axis::X, AxisDirection::POSITIVE}, {Axis::Z, AxisDirection::POSITIVE}, {1, 100} };
        }
        /** @brief OpenGL/Right-Handed: Right: X+, Forward: -Z, Up: Y+. */
        inline constexpr TransformationMeta OpenGL() {
            return { {Axis::X, AxisDirection::POSITIVE}, {Axis::Z, AxisDirection::NEGATIVE}, {Axis::Y, AxisDirection::POSITIVE} };
        }
        /** @brief ROS: Right-handed, X: Forward, Y: Left, Z: Up. */
        inline constexpr TransformationMeta ROS() {
            return { {Axis::Y, AxisDirection::NEGATIVE}, {Axis::X, AxisDirection::POSITIVE}, {Axis::Z, AxisDirection::POSITIVE} };
        }
    }
}
