#pragma once

#include "colliders/collider-base.h"

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
        m_normal = (b - a).cross(c - a);
        m_distance = -a.dot(m_normal);
    }

    inline CollisionVector checkCollision(Transform* transform, ColliderBase* collider, Transform* colliderTransform)
    {
        return collider->checkCollision(transform, this, colliderTransform);
    }

    inline CollisionVector checkCollision(Transform* transform, SphereCollider* sphereCollider, Transform* sphereTransform)
    {
        return algos::PlaneVSphereCollisionCheck(transform, this, sphereTransform, sphereCollider);
    }

    inline CollisionVector checkCollision(Transform* transform, PlaneCollider* planeCollider, Transform* planeTransform)
    {
        return {};
    }

    inline CollisionVector checkCollision(Transform* transform, CapsuleCollider* capsuleCollider, Transform* capsuleTransform)
    {
        return algos::PlaneVCapsuleCollisionCheck(transform, this, capsuleTransform, capsuleCollider);
    }

    inline vector3f furthestPoint(const vector3f& dir)
    {
        return {0};
    }

    vector3f getNormal() const
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
};

}