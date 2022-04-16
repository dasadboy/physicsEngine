#include "algos.h"

struct vec3f
{
    float m_x;
    float m_y;
    float m_z;

    vec3f()
    {
        m_x = 0;
        m_y = 0;
        m_z = 0;
    }
    
    vec3f(float n)
    {
        m_x = n;
        m_y = n;
        m_z = n;
    }

    vec3f(float x, float y, float z)
    {
        m_x = x;
        m_y = y;
        m_z = z;
    }

    vec3f operator+(vec3f& right)
    {
        return {m_x + right.m_x, m_y + right.m_y, m_z + right.m_z};
    }

    vec3f& operator+=(vec3f& right)
    {
        m_x += right.m_x;
        m_y += right.m_y;
        m_z += right.m_z;
        return *this;
    }

    vec3f operator-(vec3f& right)
    {
        return {m_x - right.m_x, m_y - right.m_y, m_z - right.m_z};
    }

    vec3f& operator-=(vec3f& right)
    {
        m_x -= right.m_x;
        m_y -= right.m_y;
        m_z -= right.m_z;
        return *this;
    }

    vec3f operator*(float right)
    {
        return {m_x * right, m_y * right, m_z * right};
    }

    vec3f operator*(float right)
    {
        m_x *= right, m_y *= right, m_z *= right;
    }

    vec3f operator/(float right)
    {
        return {m_x / right, m_y / right, m_z / right};
    }

    vec3f operator/(float right)
    {
        m_x /= right, m_y /= right, m_z /= right;
    }

    static float dotProduct(vec3f A, vec3f B)
    {
        return A.m_x * B.m_x + A.m_y * B.m_y + A.m_z * B.m_z;
    }

    static vec3f crossProduct(vec3f A, vec3f B)
    {
        return {A.m_y * B.m_z - A.m_z * B.m_y, A.m_x * B.m_z - A.m_z * B.m_x, A.m_x * B.m_y - A.m_y * B.m_x};
    }

    void normalize()
    {
        float invsqrt = ALGOS::fastInvSqrt(m_x + m_y + m_z);
        m_x *= invsqrt;
        m_y *= invsqrt;
        m_z *= invsqrt;
    }
};
