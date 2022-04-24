#pragma once

#include "math/algos.h"

namespace physics {

template < class T >
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

    inline vector3<T> operator+(vector3<T>& right)
    {
        return {m_x + right.m_x, m_y + right.m_y, m_z + right.m_z};
    }

    inline vector3<T>& operator+=(vector3<T>& right)
    {
        m_x += right.m_x;
        m_y += right.m_y;
        m_z += right.m_z;
        return *this;
    }

    inline vector3<T> operator-(vector3<T>& right)
    {
        return {m_x - right.m_x, m_y - right.m_y, m_z - right.m_z};
    }

    inline vector3<T>& operator-=(vector3<T>& right)
    {
        m_x -= right.m_x;
        m_y -= right.m_y;
        m_z -= right.m_z;
        return *this;
    }

    inline vector3<T> operator*(T right)
    {
        return {m_x * right, m_y * right, m_z * right};
    }

    inline vector3<T>& operator*=(T right)
    {
        m_x *= right, m_y *= right, m_z *= right;
        return *this;
    }

    inline vector3<T> operator/(T right)
    {
        return {m_x / right, m_y / right, m_z / right};
    }

    inline vector3<T>& operator/=(T right)
    {
        m_x /= right, m_y /= right, m_z /= right;
        return *this;
    }

    inline T dot(vector3<T> right)
    {
        return m_x * right.m_x + m_y * right.m_y + m_z * right.m_z;
    }

    inline vector3<T> cross(vector3<T> right)
    {
        return {m_y * right.m_z - m_z * right.m_y, m_x * right.m_z - m_z * right.m_x, m_x * right.m_y - m_y * right.m_x};
    }

    inline vector3<T>& normalize()
    {
        T invsqrt = ALGOS::fastInvSqrt(m_x + m_y + m_z);
        m_x *= invsqrt;
        m_y *= invsqrt;
        m_z *= invsqrt;
        return *this;
    }

    inline float magnitude()
    {
        return sqrt(dot(*this))
    }
}; // class vector3

using vector3f = vector3<float>;
using vector3i = vector3<int>;

// Stores data of possible collision
struct Collision
{
    vector3f pointA;
    vector3f pointB;
    vector3f normal; // B - A normalized
    float depth; // || B - A ||
    bool collides;
};

struct Transform
{
    vector3f pos;
    float rotation;
};

} // namespace physics
