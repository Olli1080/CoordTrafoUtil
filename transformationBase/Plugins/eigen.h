#pragma once

#include <Eigen/Core>

class Matrix4Eigen
{
public:

    typedef Eigen::Matrix4f type;

    Matrix4Eigen(Eigen::Matrix4f& matrix)
        : matrix(matrix)
    {}

    float& operator()(size_t row, size_t column);
    const float& operator()(size_t row, size_t column) const;

    static constexpr size_t size = 4;

    Eigen::Matrix4f& matrix;
};

class Matrix4EigenConst
{
public:

    typedef Eigen::Matrix4f type;

    Matrix4EigenConst(const Eigen::Matrix4f& matrix)
        : matrix(matrix)
    {}

    const float& operator()(size_t row, size_t column) const;

    static constexpr size_t size = 4;

    const Eigen::Matrix4f& matrix;
};

class Matrix3Eigen
{
public:

    typedef Eigen::Matrix3f type;

    Matrix3Eigen(Eigen::Matrix3f& matrix)
        : matrix(matrix)
    {}

    float& operator()(size_t row, size_t column);
    const float& operator()(size_t row, size_t column) const;

    static constexpr size_t size = 3;

    Eigen::Matrix3f& matrix;
};

class Matrix3EigenConst
{
public:

    typedef Eigen::Matrix3f type;

    Matrix3EigenConst(const Eigen::Matrix3f& matrix)
        : matrix(matrix)
    {}

    const float& operator()(size_t row, size_t column) const;

    static constexpr size_t size = 3;

    const Eigen::Matrix3f& matrix;
};

class Vector3Eigen
{
public:

    typedef Eigen::Vector3f type;

    Vector3Eigen(Eigen::Vector3f& vector)
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

    Eigen::Vector3f& vector;
};

class Size3Eigen
{
public:

    typedef Eigen::Vector3f type;

    Size3Eigen(Eigen::Vector3f& vector)
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

    Eigen::Vector3f& vector;
};

class QuaternionEigen
{
public:

    typedef Eigen::Quaternion<float> type;

    QuaternionEigen(Eigen::Quaternion<float>& quaternion)
        : quaternion(quaternion)
    {}

    void set_x(float x);
    [[nodiscard]] float get_x() const;

    void set_y(float y);
    [[nodiscard]] float get_y() const;

    void set_z(float z);
    [[nodiscard]] float get_z() const;

    void set_w(float w);
    [[nodiscard]] float get_w() const;

    void operator()(size_t idx, float value);
    [[nodiscard]] float operator()(size_t idx) const;

    Eigen::Quaternion<float>& quaternion;
};