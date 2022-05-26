#pragma once

#include "common/vector3.h"
#include "common/common.h"

namespace physics
{

    class Quaternion
    {
        float m_w;
        vector3f m_v;
        
    public:

        Quaternion(float w, const vector3f& v) :
        m_w(w),
        m_v(v)
        {}

        inline bool isUnitary() const
        {
            return abs(1 - m_w * m_w + m_v.dot(m_v)) < MIN_ALLOWANCE;
        }

        Quaternion operator+(const Quaternion& right) const
        {
            return {
                m_w + right.m_w,
                m_v + right.m_v
            };
        }

        Quaternion& operator+=(const Quaternion& right)
        {
            m_w += right.m_w;
            m_v += right.m_v;
            return *this;
        }

        Quaternion operator-(const Quaternion& right) const
        {
            return {
                m_w - right.m_w,
                m_v - right.m_v
            };
        }

        Quaternion& operator-=(const Quaternion& right)
        {
            m_w -= right.m_w;
            m_v -= right.m_v;
            return *this;
        }

        Quaternion operator*(const Quaternion& right) const
        {
            return {
                m_w * right.m_w - m_v.dot(right.m_v),
                m_v.cross(right.m_v) + m_v * right.m_w + right.m_v * m_w
            };
        }

        Quaternion operator*(const float right) const
        {
            return {
                m_w * right,
                m_v * right
            };
        }

        Quaternion operator*=(const float right)
        {
            m_w *= right;
            m_v *= right;
            return *this;
        }

        Quaternion operator/(const float right) const
        {
            return {
                m_w / right,
                m_v / right
            };
        }

        Quaternion operator/=(const float right)
        {
            m_w /= right;
            m_v /= right;
            return *this;
        }

        float dot(const Quaternion& q) const
        {
            return m_v.dot(q.m_v) + m_w * q.m_w;
        }   

        vector3f rotate(const vector3f& v) const
        {
            m_assert(isUnitary(), "Quaternion is not unitary.");
            return m_v * (m_w * m_w - m_v.dot(m_v)) + m_v * 2 * (m_v.dot(v)) + m_v.cross(v) * m_w * 2;
        }
    }; // class Quaternion

} // namespace physics