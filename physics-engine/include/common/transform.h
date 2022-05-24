#pragma once

#include "common/vector3.h"
#include "common/quaternion.h"

namespace physics
{

    struct Transform
    {
        vector3f pos;
        Quaternion rotation;
    };
    
}