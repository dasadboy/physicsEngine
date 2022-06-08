#pragma once

#include "common/vector3.h"
#include "common/quaternion.h"

namespace physics
{

    struct Transform
    {
        vector3f pos;
        Quaternion rotation;

        Transform(const vector3f& pos = {1}, const Quaternion& rotation = {1}) :
        pos(pos),
        rotation(rotation)
        {}

        inline void move(const vector3f& d)
        {
            pos += d;
        }

        // rotate transform by q
        inline void rotateSelf(const Quaternion& q)
        {
            rotation = q * rotation;
        }

        // rotate v
        inline vector3f rotate(const vector3f& v) const
        {
            m_assert(rotation.isUnitary(), "Quaternion is not unitary.");
            return rotation.v * (rotation.w * rotation.w - rotation.v.dot(rotation.v)) + rotation.v * 2 * (rotation.v.dot(v)) + rotation.v.cross(v) * rotation.w * 2;
        }
    };
    
}