#pragma once

#include "common/math-defines.h"

namespace physics {

template <typename T>
concept arithmeticType = std::integral<T> or std::floating_point<T>;

template < typename T >
  requires arithmeticType<T> // enforces numerical type
struct vector3
{
    T x;
    T y;
    T z;

    static const vector3f unit_x (1, 0, 0);
    static const vector3f unit_y (0, 1, 0);
    static const vector3f unit_z (0, 0, 1);

    vector3() :
    x(0),
    y(0),
    z(0)
    {}
    
    vector3(T n) :
    x(n),
    y(n),
    z(n)
    {}
    
    vector3(T x, T y, T z) :
    x(x),
    y(y),
    z(z)
    {}

    inline vector3<T> operator+(const vector3<T>& right) const
    {
        return {x + right.x, y + right.y, z + right.z};
    }

    inline vector3<T>& operator+=(const vector3<T>& right)
    {
        x += right.x;
        y += right.y;
        z += right.z;
        return *this;
    }

    inline vector3<T>& operator-()
    {
        x *= -1;
        y *= -1;
        z *= -1;
        return *this;
    }

    inline vector3<T> operator-(const vector3<T>& right) const
    {
        return {x - right.x, y - right.y, z - right.z};
    }

    inline vector3<T>& operator-=(const vector3<T>& right)
    {
        x -= right.x;
        y -= right.y;
        z -= right.z;
        return *this;
    }

    inline vector3<T> operator*(T right) const
    {
        return {x * right, y * right, z * right};
    }

    inline vector3<T>& operator*=(T right)
    {
        x *= right, y *= right, z *= right;
        return *this;
    }

    inline vector3<T> operator/(T right) const
    {
        return {x / right, y / right, z / right};
    }

    inline vector3<T>& operator/=(T right)
    {
        x /= right, y /= right, z /= right;
        return *this;
    }

    inline T dot(const vector3<T> right) const
    {
        return x * right.x + y * right.y + z * right.z;
    }

    inline vector3<T> cross(vector3<T> right) const
    {
        return {y * right.z - z * right.y, x * right.z - z * right.x, x * right.y - y * right.x};
    }

    inline vector3<T>& normalize()
    {
        T invsqrt = 1/getMagnitude();
        x *= invsqrt;
        y *= invsqrt;
        z *= invsqrt;
        return *this;
    }

    inline vector3<T> getNormal() const
    {
        T sqrt = std::sqrt(dot(*this));
        return {x / sqrt,
                y / sqrt,
                z / sqrt};
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
