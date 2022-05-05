#pragma once

#include "colliders/collider-base.h"

namespace physics
{

class SphereCollider : ColliderBase
{
private:
    float m_radius;

public:
    SphereCollider() :
    m_radius(1)
    {}

    SphereCollider(float radius) :
    m_radius(radius)
    {}

    inline CollisionVector checkCollision(Transform* transform, ColliderBase* collider, Transform* colliderTransform) override
    {
        return collider->checkCollision(colliderTransform, this, transform);
    }

    inline CollisionVector checkCollision(Transform* transform, SphereCollider* sphereCollider, Transform* sphereTransform) override
    {
        return algos::SphereVSphereCollisionCheck(transform, this, sphereTransform, sphereCollider);
    }

    inline CollisionVector checkCollision(Transform* transform, PlaneCollider* planeCollider, Transform* planeTransform) override
    {
        return algos::SphereVPlaneCollisionCheck(transform, this, planeTransform, planeCollider);
    }

    inline CollisionVector checkCollision(Transform* transform, CapsuleCollider* capsuleCollider, Transform* capsuleTransform)
    {
        return algos::SphereVCapsuleCollisionCheck(transform, this, capsuleTransform, capsuleCollider);
    }

    inline vector3f furthestPoint(const vector3f& dir) override
    {
        return {0};
    }

    inline float getRadius() const
    {
        return m_radius;
    }
};

} // physics
