#pragma once
// Optional stream-insertion helpers. Not included by TransformationHelper.h so the
// core stays free of <ostream>; include this header explicitly when you want
// `std::cout << meta` and friends. The lightweight `to_string(...)` overloads in
// types.h / meta.h are always available without this header.
#include <ostream>
#include <base-transformation/types.h>
#include <base-transformation/meta.h>

namespace Transformation
{
    inline std::ostream& operator<<(std::ostream& os, Axis a) { return os << to_string(a); }
    inline std::ostream& operator<<(std::ostream& os, AxisDirection d) { return os << to_string(d); }
    inline std::ostream& operator<<(std::ostream& os, Handedness h) { return os << to_string(h); }
    inline std::ostream& operator<<(std::ostream& os, AxisAlignment aa) { return os << to_string(aa); }
    inline std::ostream& operator<<(std::ostream& os, const TransformationMeta& m) { return os << to_string(m); }
}
