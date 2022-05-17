#pragma once

#include "math/vector3.h"

namespace physics
{
    
// Stores data of possible collision
struct CollisionVector
{
    vector3f pointA;
    vector3f pointB;
    vector3f direction; // directional vector from A to B
    float depth; // distance of furthest points

    CollisionVector() :
    pointA(0),
    pointB(0),
    direction(0),
    depth(0)
    {}
    
    CollisionVector(vector3f& A, vector3f& B) :
    pointA(A),
    pointB(B),
    direction((B - A).normalize()),
    depth((B - A).getMagnitude())
    {}

    CollisionVector(vector3f& A, vector3f& B, vector3f& dir, float depth) :
    pointA(A),
    pointB(B),
    direction(dir),
    depth(depth)
    {}

    inline bool collides() const
    {
        return depth > 0;
    }
};

}