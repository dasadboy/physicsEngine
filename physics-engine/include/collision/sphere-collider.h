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
        return {0};
    }

    virtual inline const vector3f& getRelativePosition() const {}

    virtual inline const vector3f getAbsolutePosition(const Transform& t) const {}

    inline float getRadius() const
    {
        return m_radius;
    }

}; // class SphereCollider

} // physics
