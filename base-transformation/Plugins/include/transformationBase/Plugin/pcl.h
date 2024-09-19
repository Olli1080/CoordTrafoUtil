#pragma once

#include <pcl/impl/point_types.hpp>

class Vector3PCL
{
public:

    typedef pcl::PointXYZ type;

    Vector3PCL(pcl::PointXYZ& vector)
        : vector(vector)
    {}

    void set_x(float x);
    [[nodiscard]] float get_x() const;

    void set_y(float y);
    [[nodiscard]] float get_y() const;

    void set_z(float z);
    [[nodiscard]] float get_z() const;

    void operator()(size_t idx, float value);
    [[nodiscard]] float operator()(size_t idx) const;

    pcl::PointXYZ& vector;
};