#pragma once

#include "common/basic-data-types.h"

namespace physics {

enum ColliderType
{
    SPHERE,
    PLANE,
    CAPSULE
};

class Collider
{
    vector3f m_position;
    ColliderType m_type;

public:
    Collider(const ColliderType type, const vector3f& position = {0, 0, 0}) :
    m_type(type),
    m_position(position)
    {}

    inline size_t getType() const { return (unsigned int) m_type; }

    virtual inline const vector3f furthestPoint(const vector3f& dir) const = 0;

    virtual inline const vector3f& getRelativePosition() const
    {
        return m_position;
    }

    virtual inline const vector3f getAbsolutePosition(const Transform& t) const
    {
        return t.rotate( m_position ) + t.pos;
    }
}; // class Collider

} // namespace physics
