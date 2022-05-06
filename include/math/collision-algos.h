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
        const Transform* transform1, 
        const SphereCollider* collider1, 
        const Transform* transform2, 
        const SphereCollider* collider2);

    CollisionVector SphereVPlaneCollisionCheck(
        const Transform* sphereTransform, 
        const SphereCollider* sphereCollider, 
        const Transform* planeTransform, 
        const PlaneCollider* planeCollider);

    CollisionVector PlaneVSphereCollisionCheck(
        const Transform* planeTransform, 
        const PlaneCollider* planeCollider, 
        const Transform* sphereTransform, 
        const SphereCollider* sphereCollider);

    CollisionVector CapsuleVCapsuleCollisionCheck(
        const Transform* transform1,
        const CapsuleCollider* collider1,
        const Transform* transform2,
        const CapsuleCollider* collider2);

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