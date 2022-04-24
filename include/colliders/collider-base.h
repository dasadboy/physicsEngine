#pragma once

#include "basic-data-types.h"

namespace physics {

class ColliderBase
{
public:
    virtual Collision checkCollision(Transform* transform, ColliderBase* collider, Transform* colliderTransform) = 0;

    virtual Collision checkCollision(Transform* transform, ColliderBase* sphereCollider, Transform* sphereTransform) = 0;

    virtual Collision checkCollision(Transform* transform, ColliderBase* planeCollider, Transform* planeTransform) = 0;
};

} // namespace physics
