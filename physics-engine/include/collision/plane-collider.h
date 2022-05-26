#pragma once

#include "collision/collider-base.h"

namespace physics {

class PlaneCollider : ColliderBase
{
private:
    vector3f m_normal;
    float m_distance;

public:
    PlaneCollider();

    PlaneCollider(float normalx, float normaly, float normalz, float distance) :
    m_normal(normalx, normaly, normalz),
    m_distance(distance)
    {}

    PlaneCollider(vector3f& normal, float distance) :
    m_normal(normal),
    m_distance(distance)
    {}

    PlaneCollider(vector3f& normal, vector3f& point) :
    m_normal(normal),
    m_distance(-point.dot(normal))
    {}

    PlaneCollider(vector3f& a, vector3f& b, vector3f& c)
    {
        m_normal = (b - a).cross(c - a).normalize();
        m_distance = -a.dot(m_normal);
    }

    inline vector3f furthestPoint(const vector3f& dir)
    {
        return {0};
    }

    virtual inline const vector3f& getRelativePosition() const override
    {
        m_assert(true, "PlaneCollider has no relative position. Position is defined by distance and \
        vector normal to plane");
    } 

    virtual inline vector3f getAbsolutePosition(const Transform& t) const override
    {
        m_assert(true, "PlaneCollider has no absolute position. Position is defined by distance and \
        vector normal to plane");
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



}; // class PlaneCollider

} // namespace physics