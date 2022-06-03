#pragma once

#include "common/math-defines.h"

namespace physics {

template <typename T>
concept arithmeticType = std::integral<T> or std::floating_point<T>;

template < typename T >
  requires arithmeticType<T> // enforces numerical type
struct vector3
{
    T m_x;
    T m_y;
    T m_z;

    vector3() :
    m_x(0),
    m_y(0),
    m_z(0)
    {}
    
    vector3(T n) :
    m_x(n),
    m_y(n),
    m_z(n)
    {}
    
    vector3(T x, T y, T z) :
    m_x(x),
    m_y(y),
    m_z(z)
    {}

    inline vector3<T> operator+(const vector3<T>& right) const
    {
        return {m_x + right.m_x, m_y + right.m_y, m_z + right.m_z};
    }

    inline vector3<T>& operator+=(const vector3<T>& right)
    {
        m_x += right.m_x;
        m_y += right.m_y;
        m_z += right.m_z;
        return *this;
    }

    inline vector3<T>& operator-()
    {
        m_x *= -1;
        m_y *= -1;
        m_z *= -1;
        return *this;
    }

    inline vector3<T> operator-(const vector3<T>& right) const
    {
        return {m_x - right.m_x, m_y - right.m_y, m_z - right.m_z};
    }

    inline vector3<T>& operator-=(const vector3<T>& right)
    {
        m_x -= right.m_x;
        m_y -= right.m_y;
        m_z -= right.m_z;
        return *this;
    }

    inline vector3<T> operator*(T right) const
    {
        return {m_x * right, m_y * right, m_z * right};
    }

    inline vector3<T>& operator*=(T right)
    {
        m_x *= right, m_y *= right, m_z *= right;
        return *this;
    }

    inline vector3<T> operator/(T right) const
    {
        return {m_x / right, m_y / right, m_z / right};
    }

    inline vector3<T>& operator/=(T right)
    {
        m_x /= right, m_y /= right, m_z /= right;
        return *this;
    }

    inline T dot(const vector3<T> right) const
    {
        return m_x * right.m_x + m_y * right.m_y + m_z * right.m_z;
    }

    inline vector3<T> cross(vector3<T> right) const
    {
        return {m_y * right.m_z - m_z * right.m_y, m_x * right.m_z - m_z * right.m_x, m_x * right.m_y - m_y * right.m_x};
    }

    inline vector3<T>& normalize()
    {
        T invsqrt = 1/getMagnitude();
        m_x *= invsqrt;
        m_y *= invsqrt;
        m_z *= invsqrt;
        return *this;
    }

    inline vector3<T> getNormal() const
    {
        T sqrt = std::sqrt(dot(*this));
        return {m_x / sqrt,
                m_y / sqrt,
                m_z / sqrt};
    }

    inline float getMagnitudeSquared() const
    {
        return dot(*this);
    }

    inline float getMagnitude() const
    {
        return std::sqrt(dot(*this));
    }
}; // class vector3

using vector3f = vector3<float>;
using vector3i = vector3<int>;

} // namespace physics
