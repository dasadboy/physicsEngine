#pragma once

#include "collision/collider-base.h"

namespace physics
{
class SphereCollider : Collider
{
private:
    vector3f m_position;
    float m_radius;

public:
    SphereCollider(float radius = 1, const vector3f& position = {0, 0, 0}) :
    Collider(ColliderType::SPHERE, position),
    m_radius(radius)
    {}

    inline const vector3f furthestPoint(const vector3f& dir) const override
    {
        m_assert(dir.getMagnitude() == 1, "dir is not normalized");
        return m_position + dir * m_radius;
    }

    inline float getRadius() const
    {
        return m_radius;
    }

}; // class SphereCollider

} // physics
