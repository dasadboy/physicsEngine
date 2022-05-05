#include "math/collision-algos.h"
#include "colliders/sphere-collider.h"
#include "colliders/plane-collider.h"
#include "colliders/capsule-collider.h"

namespace physics
{

namespace algos
{

    inline CollisionVector SphereVSphereCollisionCheck(
        const Transform* transform1, 
        const SphereCollider* collider1, 
        const Transform* transform2, 
        const SphereCollider* collider2)
    {
        float rad1 = collider1->getRadius(), rad2 = collider2->getRadius();
        vector3f vector12 = (transform2->pos - transform1->pos).normalize();

        vector3f A =  vector12 * rad1 + transform1->pos;
        vector3f B = -vector12 * rad2 + transform2->pos;

        return { A, B, B - A, -(B - A).dot(vector12) };
    }

    inline CollisionVector SphereVPlaneCollisionCheck(
        const Transform* sphereTransform, 
        const SphereCollider* sphereCollider, 
        const Transform* planeTransform, 
        const PlaneCollider* planeCollider)
    {
        float sphereRad = sphereCollider->getRadius();
        vector3f sphereCentre = sphereTransform->pos;
        vector3f planeNorm = planeCollider->getNormal();
        float dist = planeCollider->getDistanceFromPoint(sphereCentre);

        vector3f A = sphereCentre + planeNorm * sphereRad;
        vector3f B = sphereCentre + planeNorm * dist;

        return { A, B, (B - A).normalize(), -dist };
    }

    inline CollisionVector SphereVCapsuleCollisionCheck(
        const Transform* sphereTransform,
        const SphereCollider* sphereCollider,
        const Transform* capsuleTransform,
        const CapsuleCollider* capsuleCollider)
    {
        const vector3f& sphereCentre = sphereTransform->pos;
        const vector3f& capsuleStart = capsuleTransform->pos;
        const vector3f& capsuleVector = capsuleCollider->getVector();
        
        float t = ((sphereCentre - capsuleStart).dot(capsuleVector))/(capsuleVector.dot(capsuleVector));
        t = std::max(0.f, std::min(t, 1.f));
        
        vector3f a = capsuleVector * t + capsuleStart,
                 v = (sphereCentre - a).normalize();

        float r1 = capsuleCollider->getRadius(), r2 = sphereCollider->getRadius();

        vector3f A =  v * r1 + a;
        vector3f B = -v * r2 + sphereCentre;

        return { B, A, (A - B).normalize(), -(B - A).dot(v) };
    }

    inline CollisionVector PlaneVSphereCollisionCheck(
        const Transform* planeTransform, 
        const PlaneCollider* planeCollider, 
        const Transform* sphereTransform, 
        const SphereCollider* sphereCollider)
    {
        float sphereRad = sphereCollider->getRadius();
        const vector3f& sphereCentre = sphereTransform->pos;
        const vector3f& planeNorm = planeCollider->getNormal();
        float dist = planeCollider->getDistanceFromPoint(sphereCentre);

        vector3f A = sphereCentre + planeNorm * sphereRad;
        vector3f B = sphereCentre + planeNorm * dist;
        return { B, A, (A - B).normalize(), -dist };
    }

    inline CollisionVector PlaneVCapsuleCollisionCheck(
        const Transform* planeTransform,
        const PlaneCollider* planeCollider,
        const Transform* capsuleTransform,
        const CapsuleCollider* capsuleCollider) 
    {
        vector3f capsuleStart = capsuleTransform->pos,
                 capsuleEnd = capsuleTransform->pos + capsuleCollider->getVector();

        float distPlaneToStart = planeCollider->getDistanceFromPoint(capsuleStart);
        float distPlaneToEnd = planeCollider->getDistanceFromPoint(capsuleEnd);

        vector3f planeNormal = planeCollider->getNormal();

        vector3f closest = capsuleStart * (distPlaneToStart <= distPlaneToEnd) + capsuleEnd * (distPlaneToStart > distPlaneToEnd);

        vector3f A = closest + -planeNormal * capsuleCollider->getRadius();
        vector3f B = closest + -planeNormal * std::min(distPlaneToStart, distPlaneToEnd);

        return { B, A, (A - B), (B - A).dot(planeNormal) };
    }

    inline CollisionVector CapsuleVCapsuleCollisionCheck(
        const Transform* transform1,
        const CapsuleCollider* collider1,
        const Transform* transform2,
        const CapsuleCollider* collider2)
    {
        // adapting https://en.wikipedia.org/wiki/Skew_lines#Nearest_points
        const vector3f& v1 = collider1->getVector(), v2 = collider2->getVector();
        vector3f n = v1.cross(v2);
        vector3f n1 = v1.cross(n), n2 = v2.cross(n);
        vector3f p1 = transform1->pos, p2 = transform2->pos;
        // s or t not between 0 and 1 are out of bounds points
        float s = std::max(0.f, std::min(((p2 - p1).dot(n2))/(v1.dot(n2)), 1.f));
        float t = std::max(0.f, std::min(((p1 - p2).dot(n1))/(v2.dot(n1)), 1.f));
        // find points
        vector3f a = v1 * s + p1,
                 b = v2 * t + p2;
        vector3f v = (b - a).normalize();
        vector3f A = a + v * collider1->getRadius(),
                 B = b - v * collider2->getRadius();
        return { A, B, (B - A).normalize(), -(B - A).dot(v) };
    }

    inline CollisionVector CapsuleVSphereCollisionCheck(
        const Transform* capsuleTransform,
        const CapsuleCollider* capsuleCollider,
        const Transform* sphereTransform,
        const SphereCollider* sphereCollider)
    {
        const vector3f& sphereCentre = sphereTransform->pos;
        const vector3f& capsuleStart = capsuleTransform->pos;
        const vector3f& capsuleVector = capsuleCollider->getVector();
        
        float t = ((sphereCentre - capsuleStart).dot(capsuleVector))/(capsuleVector.dot(capsuleVector));
        t = std::max(0.f, std::min(t, 1.f));
        
        vector3f a = capsuleVector * t + capsuleStart,
                 v = (sphereCentre - a).normalize();
        float r1 = capsuleCollider->getRadius(), r2 = sphereCollider->getRadius();

        vector3f A =  v * r1 + a,
                 B = -v * r2 + sphereCentre;

        return { A, B, (B - A).normalize(), (B - A).dot(v) };
    }

    inline CollisionVector CapsuleVPlaneCollisionCheck(
        const Transform* capsuleTransform,
        const CapsuleCollider* capsuleCollider,
        const Transform* planeTransform,
        const PlaneCollider* planeCollider) 
    {
        vector3f capsuleStart = capsuleTransform->pos,
                 capsuleEnd = capsuleTransform->pos + capsuleCollider->getVector();

        float distPlaneToStart = planeCollider->getDistanceFromPoint(capsuleStart);
        float distPlaneToEnd = planeCollider->getDistanceFromPoint(capsuleEnd);

        vector3f planeNormal = planeCollider->getNormal();

        vector3f closest = capsuleStart * (distPlaneToStart <= distPlaneToEnd) + capsuleEnd * (distPlaneToStart > distPlaneToEnd);

        vector3f A = closest + -planeNormal * capsuleCollider->getRadius();
        vector3f B = closest + -planeNormal * std::min(distPlaneToStart, distPlaneToEnd);

        return { A, B, (B - A), (B - A).dot(planeNormal) };
    }

} // namespace algos

} // namespace physics
