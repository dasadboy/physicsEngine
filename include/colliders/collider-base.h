#pragma once

#include "math/basic-data-types.h"
#include "math/collision-algos.h"

namespace physics {

class ColliderBase
{
public:
    virtual inline CollisionVector checkCollision(Transform* transform, ColliderBase* collider, Transform* colliderTransform) = 0;

    virtual inline CollisionVector checkCollision(Transform* transform, SphereCollider* sphereCollider, Transform* sphereTransform) = 0;

    virtual inline CollisionVector checkCollision(Transform* transform, PlaneCollider* planeCollider, Transform* planeTransform) = 0;

    virtual inline CollisionVector checkCollision(Transform* transform, CapsuleCollider* capsuleCollider, Transform* capsuleTransform) = 0;

    virtual inline vector3f furthestPoint(const vector3f& dir) = 0;
};

} // namespace physics
