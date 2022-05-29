#pragma once

#include "collision/collider-base.h"

namespace physics
{
class SphereCollider : ColliderBase
{
private:
    vector3f m_position;
    float m_radius;

public:
    SphereCollider() :
    m_position(),
    m_radius(1)
    {}

    SphereCollider(float radius) :
    m_radius(radius)
    {}

    SphereCollider(const vector3f& pos, float radius) :
    m_position(pos),
    m_radius(radius)
    {}

    inline vector3f furthestPoint(const vector3f& dir) override
    {
        return {0};
    }

    virtual inline const vector3f& getRelativePosition() const {}

    virtual inline vector3f getAbsolutePosition(const Transform& t) const {}

    inline float getRadius() const
    {
        return m_radius;
    }

}; // class SphereCollider

} // physics
