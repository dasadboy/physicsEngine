#pragma once

#include "collision/collider-base.h"

namespace physics
{
class SphereCollider : ColliderBase
{
private:
    float m_radius;

public:
    SphereCollider() :
    m_radius(1)
    {}

    SphereCollider(float radius) :
    m_radius(radius)
    {}

    inline vector3f furthestPoint(const vector3f& dir) override
    {
        return {0};
    }

    inline float getRadius() const
    {
        return m_radius;
    }
};

} // physics
