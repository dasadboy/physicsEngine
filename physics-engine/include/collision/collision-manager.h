#pragma once

#include "collision/broadphase.h"

namespace physics
{
    enum Axis {
        X,
        Y,
        Z
    };
    
    template < Axis A >
    struct CompareObjectAABBPosition
    {
        inline bool operator()(CollisionObject* a, CollisionObject* b);
    };

    template<> bool CompareObjectAABBPosition< Axis::X >::operator()(CollisionObject* a, CollisionObject* b)
    {
        m_assert(a->isColliderAttached() && b->isColliderAttached(), "One or more arguments missing attached collider.");
        AABB& aabba = a->getCollider()->aabb;
        AABB& aabbb = b->getCollider()->aabb;
        return aabba.m_min.x < aabbb.m_min.x;
    }

    template<> bool CompareObjectAABBPosition< Axis::Y >::operator()(CollisionObject* a, CollisionObject* b)
    {
        m_assert(a->isColliderAttached() && b->isColliderAttached(), "One or more arguments missing attached collider.");
        AABB& aabba = a->getCollider()->aabb;
        AABB& aabbb = b->getCollider()->aabb;
        return aabba.m_min.y < aabbb.m_min.y;
    }

    template<> bool CompareObjectAABBPosition< Axis::Z >::operator()(CollisionObject* a, CollisionObject* b)
    {
        m_assert(a->isColliderAttached() && b->isColliderAttached(), "One or more arguments missing attached collider.");
        AABB& aabba = a->getCollider()->aabb;
        AABB& aabbb = b->getCollider()->aabb;
        return aabba.m_min.z < aabbb.m_min.z;
    }

    class CollisionManager
    {
    private:

        std::vector<CollisionObject*> x;
        std::vector<CollisionObject*> y;
        std::vector<CollisionObject*> z;

        std::unordered_set<CollisionObject*> objectsToRemove;
        size_t numObjectsRemoved;
    
    public:
        CollisionManager() :
        numObjectsRemoved(0)
        {}

        inline void addCollisionObject(CollisionObject* obj)
        {
            x.push_back(obj);
            std::sort(x.begin(), x.end(), CompareObjectAABBPosition< Axis::X >());
            y.push_back(obj);
            std::sort(y.begin(), y.end(), CompareObjectAABBPosition< Axis::Y >());
            z.push_back(obj);
            std::sort(z.begin(), z.end(), CompareObjectAABBPosition< Axis::Z >());
        }

        inline void removeCollisionObject(std::initializer_list<CollisionObject*> objs)
        {
            for (auto obj : objs)
            {
                objectsToRemove.insert(obj);
                ++numObjectsRemoved;
            }
        }

        inline void batchRemove()
        {
            auto iter = x.begin();
            auto end = x.end();
            for (; iter != end; ++iter)
            {
                if (objectsToRemove.count(*iter))
                    x.erase(iter);
            }

            iter = y.begin();
            end = y.end();

            for (; iter != end; ++iter)
            {
                if (objectsToRemove.count(*iter))
                    y.erase(iter);
            }

            iter = z.begin();
            end = z.end();
            for (; iter != end; ++iter)
            {
                if (objectsToRemove.count(*iter))
                    z.erase(iter);
            }
        }

        inline void update()
        {
            batchRemove();

            sort(x.begin(), x.end(), CompareObjectAABBPosition<Axis::X>());
            sort(y.begin(), y.end(), CompareObjectAABBPosition<Axis::Y>());
            sort(z.begin(), z.end(), CompareObjectAABBPosition<Axis::Z>());
        }

        // sphere sphere
        static inline CollisionVector getCollision(
            const Transform* transform1, 
            const SphereCollider* collider1, 
            const Transform* transform2, 
            const SphereCollider* collider2)
        {
            vector3f circle1pos = collider1->getAbsolutePosition(*transform1),
                     circle2pos = collider2->getAbsolutePosition(*transform2);
            float rad1 = collider1->getRadius(), rad2 = collider2->getRadius();
            vector3f vector12 = (circle2pos - circle1pos).normalize();

            // points on either surface closest to other shape
            vector3f A =  vector12 * rad1 + transform1->pos;
            vector3f B = -vector12 * rad2 + transform2->pos;

            vector3f dir = (B - A).normalize();

            float depth = collider2->getRadius() + collider1->getRadius() - (transform2->pos - transform1->pos).getMagnitude();

            return { A, B, dir, depth };
        }

        // sphere plane
        static inline CollisionVector getCollision(
            const Transform* sphereTransform, 
            const SphereCollider* sphereCollider, 
            const Transform* planeTransform, 
            const PlaneCollider* planeCollider)
        {
            float sphereRad = sphereCollider->getRadius();
            vector3f sphereCentre = sphereCollider->getAbsolutePosition(*sphereTransform);
            vector3f planeNorm = planeCollider->getNormal();
            float dist = planeCollider->getDistanceFromPoint(sphereCentre);

            // points on either surface closest to other shape
            vector3f A = sphereCentre + -planeNorm * sphereRad;
            vector3f B = sphereCentre + -planeNorm * dist;

            vector3f dir = (B - A).normalize();

            return { A, B, dir, -dist };
        }

        // sphere capsule
        static inline CollisionVector getCollision(
            const Transform* sphereTransform,
            const SphereCollider* sphereCollider,
            const Transform* capsuleTransform,
            const CapsuleCollider* capsuleCollider)
        {
            const vector3f sphereCentre = sphereCollider->getAbsolutePosition(*sphereTransform);
            const vector3f capsuleCentre = capsuleCollider->getAbsolutePosition(*capsuleTransform);
            const vector3f capsuleVector = capsuleTransform->rotate(capsuleCollider->getVector());
            
            float t = ((sphereCentre - capsuleCentre).dot(capsuleVector))/(capsuleVector.dot(capsuleVector));
            t = std::max(-1.f, std::min(t, 1.f));
            
            vector3f a = capsuleVector * t + capsuleCentre,
                     v = (sphereCentre - a).normalize();

            float r1 = capsuleCollider->getRadius(), r2 = sphereCollider->getRadius();

            // points on either surface closest to other shape
            vector3f A =  v * r1 + a;
            vector3f B = -v * r2 + sphereCentre;

            vector3f dir = (A - B).normalize();

            float depth = capsuleCollider->getRadius() + sphereCollider->getRadius() - (sphereCentre - a).getMagnitude();

            return { B, A, dir, depth };
        }

        // plane sphere
        static inline CollisionVector getCollision(
            const Transform* planeTransform, 
            const PlaneCollider* planeCollider, 
            const Transform* sphereTransform, 
            const SphereCollider* sphereCollider)
        {
            float sphereRad = sphereCollider->getRadius();
            const vector3f& sphereCentre = sphereCollider->getAbsolutePosition(*sphereTransform);
            const vector3f& planeNorm = planeCollider->getNormal();
            float dist = planeCollider->getDistanceFromPoint(sphereCentre);

            // points on either surface closest to other shape
            vector3f A = sphereCentre + planeNorm * sphereRad;
            vector3f B = sphereCentre + planeNorm * dist;

            vector3f dir = (A - B).normalize();

            return { B, A, dir, -dist };
        }

        // plane capsule
        static inline CollisionVector getCollision(
            const Transform* planeTransform,
            const PlaneCollider* planeCollider,
            const Transform* capsuleTransform,
            const CapsuleCollider* capsuleCollider) 
        {
            vector3f capsuleVector = capsuleTransform->rotate(capsuleCollider->getVector()),
                     capsuleCentre = capsuleCollider->getAbsolutePosition(*capsuleTransform),
                     capsuleStart = capsuleCentre - capsuleVector,
                     capsuleEnd = capsuleCentre + capsuleVector;

            float distPlaneToStart = planeCollider->getDistanceFromPoint(capsuleStart);
            float distPlaneToEnd = planeCollider->getDistanceFromPoint(capsuleEnd);

            vector3f planeNormal = planeCollider->getNormal();

            vector3f closest = capsuleStart * (distPlaneToStart <= distPlaneToEnd) + capsuleEnd * (distPlaneToStart > distPlaneToEnd);

            // points on either surface closest to other shape
            vector3f A = closest + -planeNormal * capsuleCollider->getRadius();
            vector3f B = closest + -planeNormal * std::min(distPlaneToStart, distPlaneToEnd);

            vector3f dir = (A - B).normalize();

            return { B, A, dir, (B - A).dot(planeNormal) };
        }

        // capsule capsule
        static inline CollisionVector getCollision(
            const Transform* transform1,
            const CapsuleCollider* collider1,
            const Transform* transform2,
            const CapsuleCollider* collider2)
        {
            // adapting https://en.wikipedia.org/wiki/Skew_lines#Nearest_points
            const vector3f& v1 = transform1->rotate(collider1->getVector()), v2 = transform2->rotate(collider2->getVector());
            vector3f n = v1.cross(v2);
            vector3f n1 = v1.cross(n), n2 = v2.cross(n);
            vector3f p1 = collider1->getAbsolutePosition(*transform1), p2 = collider1->getAbsolutePosition(*transform2);
            // s or t not between 0 and 1 are out of bounds points
            float s = std::max(-1.f, std::min(((p2 - p1).dot(n2))/(v1.dot(n2)), 1.f));
            float t = std::max(-1.f, std::min(((p1 - p2).dot(n1))/(v2.dot(n1)), 1.f));
            // find points
            vector3f a = v1 * s + p1,
                     b = v2 * t + p2;
            vector3f v = (b - a).normalize();

            // points on either surface closest to other shape
            vector3f A = a + v * collider1->getRadius(),
                     B = b - v * collider2->getRadius();

            vector3f dir = (B - A).normalize();

            float depth = collider1->getRadius() + collider2->getRadius() - (b - a).getMagnitude();

            return { A, B, dir, depth };
        }

        // capsule sphere
        static inline CollisionVector getCollision(
            const Transform* capsuleTransform,
            const CapsuleCollider* capsuleCollider,
            const Transform* sphereTransform,
            const SphereCollider* sphereCollider)
        {
            const vector3f sphereCentre = sphereCollider->getAbsolutePosition(*sphereTransform);
            const vector3f capsuleCentre = capsuleCollider->getAbsolutePosition(*capsuleTransform);
            const vector3f capsuleVector = capsuleTransform->rotate(capsuleCollider->getVector());
            
            float t = ((sphereCentre - capsuleCentre).dot(capsuleVector))/(capsuleVector.dot(capsuleVector));
            t = std::max(-1.f, std::min(t, 1.f));
            
            vector3f a = capsuleVector * t + capsuleCentre,
                     v = (sphereCentre - a).normalize();
            float r1 = capsuleCollider->getRadius(), r2 = sphereCollider->getRadius();

            // points on either surface closest to other shape
            vector3f A =  v * r1 + a,
                     B = -v * r2 + sphereCentre;

            vector3f dir = (B - A).normalize();

            float depth = capsuleCollider->getRadius() + sphereCollider->getRadius() - (sphereCentre - a).getMagnitude();

            return { A, B, dir, depth };
        }

        // capsule plane
        static inline CollisionVector getCollision(
            const Transform* capsuleTransform,
            const CapsuleCollider* capsuleCollider,
            const Transform* planeTransform,
            const PlaneCollider* planeCollider) 
        {
            vector3f capsuleVector = capsuleTransform->rotate(capsuleCollider->getVector()),
                     capsuleCentre = capsuleCollider->getAbsolutePosition(*capsuleTransform),
                     capsuleStart = capsuleCentre - capsuleVector,
                     capsuleEnd = capsuleCentre + capsuleVector;

            float distPlaneToStart = planeCollider->getDistanceFromPoint(capsuleStart);
            float distPlaneToEnd = planeCollider->getDistanceFromPoint(capsuleEnd);

            vector3f planeNormal = planeCollider->getNormal();

            // closest point on capsule to plane
            vector3f closest = capsuleStart * (distPlaneToStart <= distPlaneToEnd) + capsuleEnd * (distPlaneToStart > distPlaneToEnd);

            // points on either surface closest to other shape
            vector3f A = closest + -planeNormal * capsuleCollider->getRadius();
            vector3f B = closest + -planeNormal * std::min(distPlaneToStart, distPlaneToEnd);

            vector3f dir = (B - A).normalize();

            return { A, B, dir, (B - A).dot(planeNormal) };
        }
    }; // class CollisionManager

} // namespace physics
