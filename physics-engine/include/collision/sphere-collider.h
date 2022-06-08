#pragma once

#include "collision/collider-base.h"

namespace physics
{
class SphereCollider : public Collider
{
private:
    float m_radius;

public:
    SphereCollider(float radius = 1, const vector3f& position = {0, 0, 0}) :
    Collider(ColliderType::SPHERE, position),
    m_radius(radius)
    {}

    inline const vector3f furthestPoint(const vector3f& dir) const override
    {
        m_assert(std::abs(dir.getMagnitude() - 1) < MIN_ALLOWANCE, "dir is not normalized");
        return m_position + dir * m_radius;
    }

    inline float getRadius() const
    {
        return m_radius;
    }

    inline void updateAABB(const Transform& t) override
    {
        aabb.resize(getAbsolutePosition(m_position), m_radius);
    }

}; // class SphereCollider

} // physics
