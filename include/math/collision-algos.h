#pragma once

#include "math/basic-data-types.h"

namespace physics
{

class SphereCollider;

class PlaneCollider;

class CapsuleCollider;

namespace algos
{

    CollisionVector SphereVSphereCollisionCheck(
        Transform* transform1, 
        SphereCollider* collider1, 
        Transform* transform2, 
        SphereCollider* collider2);

    CollisionVector SphereVPlaneCollisionCheck(
        Transform* sphereTransform, 
        SphereCollider* sphereCollider, 
        Transform* planeTransform, 
        PlaneCollider* planeCollider);

    CollisionVector PlaneVSphereCollisionCheck(
        Transform* planeTransform, 
        PlaneCollider* planeCollider, 
        Transform* sphereTransform, 
        SphereCollider* sphereCollider);

    CollisionVector CapsuleVCapsuleCollisionCheck(
        Transform* transform1,
        CapsuleCollider* collider1,
        Transform* transform2,
        CapsuleCollider* collider2);

} // namespace algos

} // namespace physics