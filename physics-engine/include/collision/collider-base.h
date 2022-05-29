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

    inline ColliderType getType() const { return m_type; }

    virtual inline vector3f furthestPoint(const vector3f& dir) = 0;

    virtual inline const vector3f& getRelativePosition() const
    {
        return m_position;
    }

    virtual inline vector3f getAbsolutePosition(const Transform& t) const
    {
        return t.rotate( m_position ) + t.pos;
    }
}; // class Collider

} // namespace physics
