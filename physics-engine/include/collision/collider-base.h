#pragma once

#include "common/basic-data-types.h"

namespace physics {

class ColliderBase
{
    vector3f m_position;
public:
    ColliderBase(const vector3f& position = {0, 0, 0}) : m_position(position) {}

    virtual inline vector3f furthestPoint(const vector3f& dir) = 0;

    virtual inline const vector3f& getRelativePosition() const
    {
        return m_position;
    }

    virtual inline vector3f getAbsolutePosition(const Transform& t) const
    {
        return t.rotate( m_position ) + t.pos;
    }
}; // class ColliderBase

} // namespace physics
