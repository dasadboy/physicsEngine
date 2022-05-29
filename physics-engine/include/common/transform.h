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

        void move(const vector3f& d)
        {
            pos += d;
        }

        void rotateSelf(const Quaternion& q)
        {
            rotation = q * rotation;
        }

        vector3f rotate(const vector3f& v) const
        {
            m_assert(rotation.isUnitary(), "Quaternion is not unitary.");
            return rotation.m_v * (rotation.m_w * rotation.m_w - rotation.m_v.dot(rotation.m_v)) + rotation.m_v * 2 * (rotation.m_v.dot(v)) + rotation.m_v.cross(v) * rotation.m_w * 2;
        }
    };
    
}