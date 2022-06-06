#pragma once

#include "common/vector3.h"
#include "common/common.h"

namespace physics
{

    class Quaternion
    {
    public:
        
        float w;
        vector3f v;

        Quaternion(float w = 1, const vector3f& v = {0}) :
        w(w),
        v(v)
        {}

        inline bool isUnitary() const
        {
            return abs(1 - w * w + v.dot(v)) < MIN_ALLOWANCE;
        }

        inline Quaternion operator+(const Quaternion& right) const
        {
            return {
                w + right.w,
                v + right.v
            };
        }

        inline Quaternion& operator+=(const Quaternion& right)
        {
            w += right.w;
            v += right.v;
            return *this;
        }

        inline Quaternion operator-(const Quaternion& right) const
        {
            return {
                w - right.w,
                v - right.v
            };
        }

        inline Quaternion& operator-=(const Quaternion& right)
        {
            w -= right.w;
            v -= right.v;
            return *this;
        }

        inline Quaternion operator*(const Quaternion& right) const
        {
            return {
                w * right.w - v.dot(right.v),
                v.cross(right.v) + v * right.w + right.v * w
            };
        }

        inline Quaternion operator*(const float right) const
        {
            return {
                w * right,
                v * right
            };
        }

        inline Quaternion operator*=(const float right)
        {
            w *= right;
            v *= right;
            return *this;
        }

        inline Quaternion operator/(const float right) const
        {
            return {
                w / right,
                v / right
            };
        }

        inline Quaternion operator/=(const float right)
        {
            w /= right;
            v /= right;
            return *this;
        }

        inline float dot(const Quaternion& q) const
        {
            return v.dot(q.v) + w * q.w;
        }   
    }; // class Quaternion

} // namespace physics