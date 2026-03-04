#pragma once
#include <pcl/point_types.h>
#include "base-transformation/traits.h"

namespace Transformation
{
    template<>
    struct VectorTraits<pcl::PointXYZ, float> {
        static float get_x(const pcl::PointXYZ& v) { return v.x; }
        static float get_y(const pcl::PointXYZ& v) { return v.y; }
        static float get_z(const pcl::PointXYZ& v) { return v.z; }
        static float get_idx(const pcl::PointXYZ& v, size_t i) { return v.data[i]; }

        static void set_x(pcl::PointXYZ& v, float val) { v.x = val; }
        static void set_y(pcl::PointXYZ& v, float val) { v.y = val; }
        static void set_z(pcl::PointXYZ& v, float val) { v.z = val; }
        static void set_idx(pcl::PointXYZ& v, size_t i, float val) { v.data[i] = val; }
    };
}
