#pragma once

#include "collision/collider-base.h"

namespace physics {

class PlaneCollider : public Collider
{
private:
    vector3f m_normal;
    float m_distance;

public:
    PlaneCollider(float normalx, float normaly, float normalz, float distance) :
    Collider(ColliderType::PLANE),
    m_normal(normalx, normaly, normalz),
    m_distance(distance)
    {}

    PlaneCollider(const vector3f& normal = {1, 0, 0}, float distance = 0) :
    Collider(ColliderType::PLANE),
    m_normal(normal),
    m_distance(distance)
    {}

    PlaneCollider(vector3f& normal, vector3f& point) :
    Collider(ColliderType::PLANE),
    m_normal(normal),
    m_distance(-point.dot(normal))
    {}

    PlaneCollider(vector3f& a, vector3f& b, vector3f& c) :
    Collider(ColliderType::PLANE)
    {
        m_normal = (b - a).cross(c - a).normalize();
        m_distance = -a.dot(m_normal);
    }

    inline const vector3f furthestPoint(const vector3f& dir) const override
    {
        return {0};
    }

    inline const vector3f& getRelativePosition() const
    {
        m_assert(true, "PlaneCollider has no relative position. Position is defined by distance and \
        vector normal to plane");
        return m_position;
    } 

    inline const vector3f getAbsolutePosition(const Transform& t) const
    {
        m_assert(true, "PlaneCollider has no absolute position. Position is defined by distance and \
        vector normal to plane");
        return {0};
    }

    inline const vector3f& getNormal() const
    {
        return m_normal;
    }

    float getDistance() const
    {
        return m_distance;
    }

    float getDistanceFromPoint(const vector3f& point) const
    {
        return point.dot(m_normal) + m_distance;
    }

    inline void updateAABB(const Transform& t) override {}

}; // class PlaneCollider

} // namespace physics