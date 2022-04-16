#include "algos.h"

template < typename T >
struct vector3
{
    T m_x;
    T m_y;
    T m_z;

    vector3()
    {
        m_x = 0;
        m_y = 0;
        m_z = 0;
    }
    
    vector3(T n)
    {
        m_x = n;
        m_y = n;
        m_z = n;
    }

    vector3(T x, T y, T z)
    {
        m_x = x;
        m_y = y;
        m_z = z;
    }

    vector3<T> operator+(vector3<T>& right)
    {
        return {m_x + right.m_x, m_y + right.m_y, m_z + right.m_z};
    }

    vector3<T>& operator+=(vector3<T>& right)
    {
        m_x += right.m_x;
        m_y += right.m_y;
        m_z += right.m_z;
        return *this;
    }

    vector3<T> operator-(vector3<T>& right)
    {
        return {m_x - right.m_x, m_y - right.m_y, m_z - right.m_z};
    }

    vector3<T>& operator-=(vector3<T>& right)
    {
        m_x -= right.m_x;
        m_y -= right.m_y;
        m_z -= right.m_z;
        return *this;
    }

    vector3<T> operator*(T right)
    {
        return {m_x * right, m_y * right, m_z * right};
    }

    vector3<T> operator*(T right)
    {
        m_x *= right, m_y *= right, m_z *= right;
    }

    vector3<T> operator/(T right)
    {
        return {m_x / right, m_y / right, m_z / right};
    }

    vector3<T> operator/(T right)
    {
        m_x /= right, m_y /= right, m_z /= right;
    }

    static T dotProduct(vector3<T> A, vector3<T> B)
    {
        return A.m_x * B.m_x + A.m_y * B.m_y + A.m_z * B.m_z;
    }

    static vector3<T> crossProduct(vector3<T> A, vector3<T> B)
    {
        return {A.m_y * B.m_z - A.m_z * B.m_y, A.m_x * B.m_z - A.m_z * B.m_x, A.m_x * B.m_y - A.m_y * B.m_x};
    }

    void normalize()
    {
        T invsqrt = ALGOS::fastInvSqrt(m_x + m_y + m_z);
        m_x *= invsqrt;
        m_y *= invsqrt;
        m_z *= invsqrt;
    }
};

struct vector3f : vector3<float> {};

struct vector3i : vector3<int> {};

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
