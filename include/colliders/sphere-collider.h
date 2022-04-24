#pragma once

#include "colliders/collider-base.h"

namespace physics
{

class SphereCollider : ColliderBase
{
private:
    vector3f m_centre;
    float m_radius;

public:
    SphereCollider() :
    m_centre(0),
    m_radius(1)
    {}

    SphereCollider(vector3f centre, float radius) :
    m_centre(centre),
    m_radius(radius)
    {}

    SphereCollider(vector3f p1, vector3f p2, vector3f p3, vector3f p4)
    {
        #define U(a,b,c,d,e,f,g,h) (a.m_z - b.m_z)*(c.m_x*d.m_y - d.m_x*c.m_y) - (e.m_z - f.m_z)*(g.m_x*h.m_y - h.m_x*g.m_y)
        #define D(x,y,a,b,c) (a.x*(b.y-c.y) + b.x*(c.y-a.y) + c.x*(a.y-b.y))
        #define E(x,y) ((ra*D(x,y,p2,p3,p4) - rb*D(x,y,p3,p4,p1) + rc*D(x,y,p4,p1,p2) - rd*D(x,y,p1,p2,p3)) / uvw)
            float u = U(p1,p2,p3,p4,p2,p3,p4,p1);
            float v = U(p3,p4,p1,p2,p4,p1,p2,p3);
            float w = U(p1,p3,p4,p2,p2,p4,p1,p3);
            float uvw = 2 * (u + v + w);
            if (uvw == 0.0) {
                m_centre = 0;

            }
            float ra = p1.dot(p1);
            float rb = p2.dot(p2);
            float rc = p3.dot(p3);
            float rd = p4.dot(p4);
            float x0 = E(m_y, m_z);
            float y0 = E(m_z, m_x);
            float z0 = E(m_x, m_y);
            vector3f diff = (p1.m_x - x0, p1.m_y - y0, p1.m_z - z0);
            float dot = diff.dot(diff);
            float radius = dot * dot;
            m_centre = {x0, y0, z0};
            m_radius = radius;
        #undef U
        #undef D
        #undef E
    }

    SphereCollider(vector3f centre, vector3f point) :
    m_centre(centre),
    m_radius((centre - point).magnitude())
    {}

    Collision checkCollision(Transform* transform, ColliderBase* collider, Transform* colliderTransform);

    Collision checkCollision(Transform* transform, ColliderBase* sphereCollider, Transform* sphereTransform);

    Collision checkCollision(Transform* transform, ColliderBase* planeCollider, Transform* planeTransform);
};

} // physics
