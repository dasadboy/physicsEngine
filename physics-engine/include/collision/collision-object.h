#pragma once

#include "collision/collision-manager.h"

namespace physics
{

class CollisionObject
{
private:
    ColliderType m_type;
    Transform m_transform;
    Collider* m_collider;

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

    ~CollisionObject()
    {
        delete m_collider;
    }
};

}
