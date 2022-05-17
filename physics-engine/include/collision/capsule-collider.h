#pragma once

#include "collision/collider-base.h"

namespace physics
{

class CapsuleCollider : ColliderBase
{
private:
    // start point is {0, 0, 0}, end is m_halfVector;
    vector3f m_halfVector;
    float m_radius;

public:
    CapsuleCollider():
    m_halfVector(1, 0, 0),
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

    inline const vector3f getVector() const
    {
        return m_halfVector;
    }

    inline const float getRadius() const
    {
        return m_radius;
    }

    

};

}
