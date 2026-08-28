#pragma once
#include <cassert>
#include <pcl/point_types.h>
#include <base-transformation/traits.h>

namespace Transformation
{
    template<> struct TraitsEnabled<pcl::PointXYZ> : std::true_type {};

    template<typename T>
    struct VectorTraits<pcl::PointXYZ, T> {
        using type = pcl::PointXYZ;
        static T get_x(const type& v) { return static_cast<T>(v.x); }
        static T get_y(const type& v) { return static_cast<T>(v.y); }
        static T get_z(const type& v) { return static_cast<T>(v.z); }
        // data[3] is the homogeneous component, not a spatial axis.
        static T get_idx(const type& v, size_t i) { assert(i < 3); return static_cast<T>(v.data[i]); }
        static void set_x(type& v, T val) { v.x = static_cast<float>(val); }
        static void set_y(type& v, T val) { v.y = static_cast<float>(val); }
        static void set_z(type& v, T val) { v.z = static_cast<float>(val); }
        static void set_idx(type& v, size_t i, T val) { assert(i < 3); v.data[i] = static_cast<float>(val); }
    };
}
