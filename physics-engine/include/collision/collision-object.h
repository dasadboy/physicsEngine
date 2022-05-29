#pragma once

#include "collision/collision-manager.h"

namespace physics
{

class CollisionObject
{
private:
    ColliderType m_type;
    Collider* m_collider;

public:
    CollisionObject(ColliderType type) :
    m_type(type)
    {}

    void attachCollider(Collider* collider)
    {   
        assert(collider->getType() == m_type);
        m_collider = collider;
    }

    ~CollisionObject()
    {
        delete m_collider;
    }
};

}
