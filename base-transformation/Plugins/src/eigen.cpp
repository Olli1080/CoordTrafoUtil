#include "eigen.h"

#include <Eigen/Geometry>

void QuaternionEigen::set_x(float x)
{
    quaternion.x() = x;
}

float QuaternionEigen::get_x() const
{
    return quaternion.x();
}

void QuaternionEigen::set_y(float y)
{
    quaternion.y() = y;
}

float QuaternionEigen::get_y() const
{
    return quaternion.y();
}

void QuaternionEigen::set_z(float z)
{
    quaternion.z() = z;
}

float QuaternionEigen::get_z() const
{
    return quaternion.z();
}

void QuaternionEigen::set_w(float w)
{
    quaternion.w() = w;
}

float QuaternionEigen::get_w() const
{
    return quaternion.w();
}

void QuaternionEigen::operator()(size_t idx, float value)
{
    quaternion.coeffs().coeffRef(idx) = value;
}

float QuaternionEigen::operator()(size_t idx) const
{
    return quaternion.coeffs().coeffRef(idx);
}

float& Matrix4Eigen::operator()(size_t row, size_t column)
{
    //maybe include range check
    return matrix(row, column);
}
const float& Matrix4Eigen::operator()(size_t row, size_t column) const
{
    //maybe include range check
    return matrix(row, column);
}

void Vector3Eigen::set_x(float x)
{
    vector.x() = x;
}

float Vector3Eigen::get_x() const
{
    return vector.x();
}

void Vector3Eigen::set_y(float y)
{
    vector.y() = y;
}

float Vector3Eigen::get_y() const
{
    return vector.y();
}

void Vector3Eigen::set_z(float z)
{
    vector.z() = z;
}

float Vector3Eigen::get_z() const
{
    return vector.z();
}

void Vector3Eigen::operator()(size_t idx, float value)
{
    vector(idx, 0) = value;
}

float Vector3Eigen::operator()(size_t idx) const
{
    return vector(idx, 0);
}

void Size3Eigen::set_x(float x)
{
    vector.x() = x;
}

float Size3Eigen::get_x() const
{
    return vector.x();
}

void Size3Eigen::set_y(float y)
{
    vector.y() = y;
}

float Size3Eigen::get_y() const
{
    return vector.y();
}

void Size3Eigen::set_z(float z)
{
    vector.z() = z;
}

float Size3Eigen::get_z() const
{
    return vector.z();
}

void Size3Eigen::operator()(size_t idx, float value)
{
    vector(idx, 0) = value;
}

float Size3Eigen::operator()(size_t idx) const
{
    return vector(idx, 0);
}

float& Matrix3Eigen::operator()(size_t row, size_t column)
{
    return matrix(row, column);
}

const float& Matrix3Eigen::operator()(size_t row, size_t column) const
{
    return matrix(row, column);
}

const float& Matrix3EigenConst::operator()(size_t row, size_t column) const
{
    return matrix(row, column);
}

const float& Matrix4EigenConst::operator()(size_t row, size_t column) const
{
    return matrix(row, column);
}
