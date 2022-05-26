#pragma once

#include "collision/collider-base.h"

namespace physics
{

class CapsuleCollider : ColliderBase
{
private:
    // capsule where line is defined from m_position - m_halfVector to m_position + m_halfVector relative to transform
    vector3f m_halfVector;
    vector3f m_position;
    float m_radius;

public:
    CapsuleCollider(const vector3f& position = {0, 0, 0}) :
    ColliderBase(position),
    m_halfVector(),
    m_radius(1)
    {}

    CapsuleCollider(const vector3f& direction, float halfVectorMagnitude, float radius, const vector3f& position = {0, 0, 0}) :
    ColliderBase(position),
    m_halfVector(direction * halfVectorMagnitude),
    m_radius(radius)
    {}

    CapsuleCollider(const vector3f& vec, float radius, const vector3f& position = {0, 0, 0}) :
    ColliderBase(position),
    m_halfVector(vec),
    m_radius(radius)
    {}

    CapsuleCollider(const vector3f& a, const vector3f& b, float radius, const vector3f& position = {0, 0, 0}) :
    ColliderBase(position),
    m_halfVector((b - a)/2),
    m_radius(radius)
    {}

    
    inline vector3f furthestPoint(const vector3f& dir)
    {
        return {0};
    }

    virtual inline const vector3f& getRelativePosition() const {}

    virtual inline vector3f getAbsolutePosition(const Transform& t) const {}

    inline const vector3f& getVector() const
    {
        return m_halfVector;
    }

    inline const float getRadius() const
    {
        return m_radius;
    }

}; // class CapsuleCollider

} // namespace physics
