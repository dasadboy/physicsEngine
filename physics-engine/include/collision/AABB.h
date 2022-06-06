#include "common/basic-data-types.h"

namespace physics
{
    class AABB
    {
    private:
        vector3f m_min;
        vector3f m_max;
    public:
        AABB(const vector3f& min = {-FLT_MAX}, const vector3f& max = {FLT_MAX}) :
        m_min(min),
        m_max(max)
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
            return (m_max.x < other.m_min.x) || (m_min.x > other.m_max.x);
        }

        inline bool checkIntersectsY(const AABB& other)
        {
            return (m_max.y < other.m_min.y) || (m_min.y > other.m_max.y);
        }

        inline bool checkIntersectsZ(const AABB& other)
        {
            return (m_max.z < other.m_min.z) || (m_min.z > other.m_max.z);
        }
    };
}
