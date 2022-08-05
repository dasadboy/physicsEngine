#pragma once

#include "common/common.h"

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

    static const vector3<T> unit_x;
    static const vector3<T> unit_y;
    static const vector3<T> unit_z;

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

    template<typename To>
        requires arithmeticType<To>
    operator vector3<To>()
    {
        return {(To) x, (To) y, (To) z};
    }

    inline T operator[](int idx) const
    {
        assert(idx < 3);
        switch(idx) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
        }
        // should never come here
        return INFINITY;
    }

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

    // inline vector3<T> operator/(vector3<T> right)
    // {
    //     return {x/right.x, y/right.y, z/right.z};
    // }

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

    static inline vector3<T> accumulate_min(vector3<T> l, vector3<T> r)
    {
        return {std::min(l.x, r.x), std::min(l.y, r.y), std::min(l.z, r.z)};
    }

    static inline vector3<T> accumulate_max(vector3<T> l, vector3<T> r)
    {
        return {std::max(l.x, r.x), std::max(l.y, r.y), std::max(l.z, r.z)};
    }

    static inline vector3<T> accumulate_min_element(std::initializer_list<vector3<T>> lst)
    {
        vector3<T> ret = lst[0];
        for (auto iter = lst.begin() + 1, end = lst.end(); iter != end; ++lst)
        {
            ret.x = std::min(ret.x, iter->x);
            ret.y = std::min(ret.y, iter->y);
            ret.z = std::min(ret.z, iter->z);
        }
        return ret;
    }

    static inline vector3<T> accumulate_max_element(std::initializer_list<vector3<T>> lst)
    {
        vector3<T> ret = lst[0];
        for (auto iter = lst.begin() + 1, end = lst.end(); iter != end; ++lst)
        {
            ret.x = std::max(ret.x, iter->x);
            ret.y = std::max(ret.y, iter->y);
            ret.z = std::max(ret.z, iter->z);
        }
        return ret;
    }

}; // class vector3

using vector3f = vector3<float>;
using vector3i = vector3<int>;
using vector3s = vector3<size_t>;
using vector3ll = vector3<long long>;
using vector3d = vector3<double>;

template<typename T> const vector3<T> vector3<T>::unit_x (1, 0, 0);
template<typename T> const vector3<T> vector3<T>::unit_y (0, 1, 0);
template<typename T> const vector3<T> vector3<T>::unit_z (0, 0, 1);



} // namespace physics
