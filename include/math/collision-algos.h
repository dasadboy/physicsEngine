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

    CollisionVector CapsuleVSphereCollisionCheck(
        const Transform* capsuleTransform,
        const CapsuleCollider* capsuleCollider,
        const Transform* sphereTransform,
        const SphereCollider* sphereCollider);

    CollisionVector SphereVCapsuleCollisionCheck(
        const Transform* sphereTransform,
        const SphereCollider* sphereCollider,
        const Transform* capsuleTransform,
        const CapsuleCollider* capsuleCollider);

    CollisionVector CapsuleVPlaneCollisionCheck(
        const Transform* capsuleTransform,
        const CapsuleCollider* capsuleCollider,
        const Transform* planeTransform,
        const PlaneCollider* planeCollider);

    CollisionVector PlaneVCapsuleCollisionCheck(
        const Transform* planeTransform,
        const PlaneCollider* planeCollider,
        const Transform* capsuleTransform,
        const CapsuleCollider* capsuleCollider);

} // namespace algos

} // namespace physics