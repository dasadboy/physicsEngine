#pragma once

#include "collision/collider-base.h"

namespace physics
{

class CapsuleCollider : ColliderBase
{
private:
    // start point is {0, 0, 0}, end is m_vector;
    vector3f m_vector;
    float m_radius;

public:
    CapsuleCollider():
    m_vector(1, 0, 0),
    m_radius(1)
    {}

    CapsuleCollider(const vector3f& direction, float vectorMagnitude, float radius) :
    m_vector(direction * vectorMagnitude),
    m_radius(radius)
    {}

    CapsuleCollider(const vector3f& vec, float radius) :
    m_vector(vec),
    m_radius(radius)
    {}
    
    inline vector3f furthestPoint(const vector3f& dir)
    {
        return {0};
    }

    inline const vector3f getVector() const
    {
        return m_vector;
    }

    inline const float getRadius() const
    {
        return m_radius;
    }

    

};

}
