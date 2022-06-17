#pragma once

#include "collision/AABB.h"

namespace physics {

enum ColliderType
{
    SPHERE,
    PLANE,
    CAPSULE
};

class Collider
{
protected:
    vector3f m_position;
    ColliderType m_type;

public:
    AABB aabb;

    Collider(const ColliderType type, const vector3f& position = {0, 0, 0}) :
    m_type(type),
    m_position(position)
    {}

    inline size_t getType() const { return m_type; }

    virtual inline const vector3f furthestPoint(const vector3f& dir) const = 0;

    inline const vector3f& getRelativePosition() const
    {
        return m_position;
    }

    inline const vector3f getAbsolutePosition(const Transform& t) const
    {
        return t.rotate( m_position ) + t.pos;
    }

    virtual inline void updateAABB(const Transform& t) = 0;

}; // class Collider

} // namespace physics
