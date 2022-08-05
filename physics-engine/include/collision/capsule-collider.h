#pragma once

#include "collision/collider-base.h"

namespace physics
{

class CapsuleCollider : public Collider
{
private:
    // capsule where line is defined from m_position - m_halfVector to m_position + m_halfVector relative to transform
    vector3f m_halfVector;
    float m_radius;

public:
    CapsuleCollider(const vector3f& position = {0, 0, 0}) :
    Collider(ColliderType::CAPSULE, position),
    m_halfVector(),
    m_radius(1)
    {}

    CapsuleCollider(const vector3f& direction, float halfVectorMagnitude, float radius, const vector3f& position = {0, 0, 0}) :
    Collider(ColliderType::CAPSULE, position),
    m_halfVector(direction * halfVectorMagnitude),
    m_radius(radius)
    {}

    CapsuleCollider(const vector3f& vec, float radius, const vector3f& position = {0, 0, 0}) :
    Collider(ColliderType::CAPSULE, position),
    m_halfVector(vec),
    m_radius(radius)
    {}

    CapsuleCollider(const vector3f& a, const vector3f& b, float radius, const vector3f& position = {0, 0, 0}) :
    Collider(ColliderType::CAPSULE, position),
    m_halfVector((b - a)/2.f),
    m_radius(radius)
    {}

    
    inline const vector3f furthestPoint(const vector3f& dir) const override
    {
        m_assert(false, "not yet implemented");
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

    inline void updateAABB(const Transform& t) override
    {
        vector3f extent(m_radius, m_radius, m_radius);
        vector3f pos = getAbsolutePosition(t);
        vector3f v = t.rotate(m_halfVector);
        aabb.resize(vector3f::accumulate_min(pos + v, pos - v), vector3f::accumulate_max(pos + v, pos - v));
    }

}; // class CapsuleCollider

} // namespace physics
