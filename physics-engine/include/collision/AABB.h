#include "common/basic-data-types.h"

namespace physics
{
    class AABB
    {
    public:
        vector3f m_min;
        vector3f m_max;

        AABB(const vector3f& min = {-FLT_MAX}, const vector3f& max = {FLT_MAX}) :
        m_min(min),
        m_max(max)
        {
            assert(min.x < max.x && min.y < max.y && min.z < max.z);
        }

        AABB(const vector3f& centre, float scale) :
        m_min(centre - vector3f(scale, scale, scale)),
        m_max(centre + vector3f(scale, scale, scale))
        {}

        inline bool checkIntersects(const AABB& other)
        {
            return 
                !((m_max.x < other.m_min.x) || (m_min.x > other.m_max.x) ||
                (m_max.y < other.m_min.y) || (m_min.y > other.m_max.y) ||
                (m_max.z < other.m_min.z) || (m_min.z > other.m_max.z));
        }

        inline bool checkIntersectsX(const AABB& other)
        {
            return !((m_max.x < other.m_min.x) || (m_min.x > other.m_max.x));
        }

        inline bool checkIntersectsY(const AABB& other)
        {
            return !((m_max.y < other.m_min.y) || (m_min.y > other.m_max.y));
        }

        inline bool checkIntersectsZ(const AABB& other)
        {
            return !((m_max.z < other.m_min.z) || (m_min.z > other.m_max.z));
        }

        inline void resize(const vector3f& centre, float scale) 
        {
            m_min = centre - vector3f(scale, scale, scale);
            m_max = centre + vector3f(scale, scale, scale);
        }

        inline void resize(const vector3f& min, const vector3f& max)
        {
            assert(min.x < max.x && min.y < max.y && min.z < max.z);
            m_min = min;
            m_max = max;
        }

        inline float calcMinDistanceSquared(const AABB& b)
        {
            const AABB& a = *this;

            #define checkOverlap(d, a, b) ( !((a.m_max.d < b.m_min.d) || (a.m_min.d > b.m_max.d)) )
            #define calc(d, a, b) ( std::abs( a.m_min.d - b.m_max.d )  )

            // find min distance of each component ( 0 if overlapping on that component component)
            float diffX = checkOverlap(x, a, b) * ( std::min( calc(x, a, b), calc(x, b, a) ) );
            float diffY = checkOverlap(y, a, b) * ( std::min( calc(y, a, b), calc(y, b, a) ) );
            float diffZ = checkOverlap(z, a, b) * ( std::min( calc(z, a, b), calc(z, b, a) ) );

            #undef checkOverlap
            #undef calc

            return diffX * diffX + diffY * diffY + diffZ * diffZ;
        }
    };
}
