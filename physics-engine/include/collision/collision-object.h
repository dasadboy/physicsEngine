#pragma once

#include "capsule-collider.h"
#include "sphere-collider.h"
#include "plane-collider.h"

namespace physics
{

class CollisionObject
{
protected:
    ColliderType m_type;
    Transform m_transform;
    Collider* m_collider;

    using callbackFunc_t = std::function<void(CollisionVector, float)>;

public:
    CollisionObject(ColliderType type) :
    m_type(type),
    m_collider(nullptr)
    {}

    void attachCollider(Collider* collider)
    {   
        assert(collider->getType() == m_type);
        m_collider = collider;
    }

    void setTransform(const Transform& transform)
    {
        m_transform = transform;
    }

    void move(const vector3f& d)
    {
        m_transform.move(d);
    }

    void rotate(const Quaternion& q)
    {
        m_transform.rotateSelf(q);
    }

    ~CollisionObject()
    {
        delete m_collider;
    }
};

}
