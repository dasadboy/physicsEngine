#pragma once

#include "common/basic-data-types.h"

namespace physics {

class ColliderBase
{
public:
    virtual inline vector3f furthestPoint(const vector3f& dir) = 0;
};

} // namespace physics
