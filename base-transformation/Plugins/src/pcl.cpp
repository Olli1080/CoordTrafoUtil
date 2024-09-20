#include "pcl.h"

void Vector3PCL::set_x(float x)
{
    vector.x = x;
}

float Vector3PCL::get_x() const
{
    return vector.x;
}

void Vector3PCL::set_y(float y)
{
    vector.y = y;
}

float Vector3PCL::get_y() const
{
    return vector.y;
}

void Vector3PCL::set_z(float z)
{
    vector.z = z;
}

float Vector3PCL::get_z() const
{
    return vector.z;
}

void Vector3PCL::operator()(size_t idx, float value)
{
    vector.data[idx] = value;
}

float Vector3PCL::operator()(size_t idx) const
{
    return vector.data[idx];
}