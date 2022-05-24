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
    CapsuleCollider():
    m_halfVector(),
    m_position(),
    m_radius(1)
    {}

    CapsuleCollider(const vector3f& direction, float halfVectorMagnitude, float radius) :
    m_halfVector(direction * halfVectorMagnitude),
    m_radius(radius)
    {}

    CapsuleCollider(const vector3f& vec, float radius) :
    m_halfVector(vec),
    m_radius(radius)
    {}

    CapsuleCollider(const vector3f& a, const vector3f& b, float radius) :
    m_halfVector((b - a)/2),
    m_radius(radius)
    {}

    
    inline vector3f furthestPoint(const vector3f& dir)
    {
        return {0};
    }

    inline const vector3f& getVector() const
    {
        return m_halfVector;
    }

    inline const float getRadius() const
    {
        return m_radius;
    }

    inline const vector3f& getPosition() const
    {
        return m_position;
    }

};

}
