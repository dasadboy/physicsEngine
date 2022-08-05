#pragma once

#include "collision/collision-object.h"

namespace physics
{

    class BroadphaseSimplex
    {
        
    public:
        const size_t id;
        const AABB& aabb;

        BroadphaseSimplex(size_t id, const AABB& aabb) :
        id(id),
        aabb(aabb)
        {}
    };

    template< unsigned int dim >
    class BroadphaseSimplexCmp
    {
    public:
        bool operator()(BroadphaseSimplex* s1, BroadphaseSimplex* s2)
        {
            return s1->aabb.m_min[ dim ] < s2->aabb.m_min[ dim ];
        }
    };

    class BroadphasePairs
    {
    public:
        const size_t objA;
        const size_t objB;
    };

    class BroadphaseMbpRegion
    {
        #define DEFAULTLOC UINT_MAX
        std::vector<BroadphaseSimplex*> m_objects;
        std::vector<unsigned int> m_objLocations;
        std::vector<BroadphasePairs> m_pairs;
    
    public:

        BroadphaseMbpRegion() :
        m_objLocations(UINT_MAX, DEFAULTLOC)
        {}

        void addObject(BroadphaseSimplex* simp);

        void removeObject(BroadphaseSimplex* simp);

        void generatePairs();

        void sap0(std::vector<BroadphaseSimplex*>::iterator begin, 
        std::vector<BroadphaseSimplex*>::iterator end, size_t sz, int consecutiveNonSplits);

        void sap1(std::vector<BroadphaseSimplex*>::iterator begin, 
        std::vector<BroadphaseSimplex*>::iterator end, size_t sz, int consecutiveNonSplits);

        void sap2(std::vector<BroadphaseSimplex*>::iterator begin, 
        std::vector<BroadphaseSimplex*>::iterator end, size_t sz, int consecutiveNonSplits);

        #undef DEFAULTLOC
    };

    class BroadphaseMbp
    {
    private:

        std::unordered_map< long long, std::unordered_map< long long, std::unordered_map< long long, BroadphaseMbpRegion > > > m_regions;
        vector3f m_cellsize;
        vector3f m_updatedRegions;

    public:

        BroadphaseMbp(const vector3f& cellsize) :
        m_cellsize(cellsize)
        {}

        const BroadphaseMbpRegion& operator[](vector3ll& idx)
        {
            return m_regions[idx.x][idx.y][idx.z];
        }

        const bool count(vector3ll& idx) const;

        const BroadphaseMbpRegion& getRegionFromPosition(vector3f& pos)
        {
            vector3ll region = {static_cast<long long int>(pos.x/m_cellsize.x), 
            static_cast<long long int>(pos.y/m_cellsize.y), static_cast<long long int>(pos.z/m_cellsize.z)};
            return m_regions[region.x][region.y][region.z];
        }

        void addObject(CollisionObject* obj);
    };

}
