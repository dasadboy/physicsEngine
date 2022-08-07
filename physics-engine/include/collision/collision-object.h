#pragma once

#include "collision/sphere-collider.h"
#include "collision/capsule-collider.h"
#include "collision/plane-collider.h"

namespace physics
{

class CollisionObject
{
protected:
    ColliderType m_type;
    Transform m_transform;
    Collider* m_collider;
    ObjectHandle m_id;

    using callbackFunc_t = std::function<void(CollisionVector, float)>;

public:
    CollisionObject(ColliderType type, size_t id) :
    m_type(type),
    m_collider(nullptr),
    m_id(id)
    {}

    ObjectHandle getid() const
    {
        return m_id;
    }

    void attachCollider(Collider* collider)
    {   
        assert(collider->getType() == m_type);
        m_collider = collider;
    }

    void detachCollider()
    {
        m_collider = nullptr;
    }

    bool isColliderAttached()
    {
        return m_collider != nullptr;
    }

    Collider* getCollider() { return m_collider; }

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

    void updateAABB()
    {
        m_collider->updateAABB(m_transform);
    }

    const AABB& getAABB() const
    {
        return m_collider->aabb;
    }

    ~CollisionObject()
    {
        delete m_collider;
    }
};

}
