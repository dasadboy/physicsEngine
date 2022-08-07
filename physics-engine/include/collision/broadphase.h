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


    class BroadphasePairs
    {
    public:
        const size_t objA;
        const size_t objB;
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

        void addSimplex(BroadphaseSimplex* simp);

        void removeSimplex(BroadphaseSimplex* simp);

        bool isEmpty()
        {
            return !m_objects.size();
        }

        void generatePairs();

        void sap0(std::vector<BroadphaseSimplex*>::iterator begin, 
            std::vector<BroadphaseSimplex*>::iterator end, 
            size_t sz, int consecutiveNonSplits);

        void sap1(std::vector<BroadphaseSimplex*>::iterator begin, 
            std::vector<BroadphaseSimplex*>::iterator end, 
            size_t sz, int consecutiveNonSplits);

        void sap2(std::vector<BroadphaseSimplex*>::iterator begin, 
            std::vector<BroadphaseSimplex*>::iterator end, 
            size_t sz, int consecutiveNonSplits);

        #undef DEFAULTLOC
    };


    class BroadphaseHash 
    {
        static const long long mod;
    public:

        size_t operator()(vector3i& key)
        {
            return ((key.x % mod)* 2467 + (key.y % mod) * 71167 + (key.z % mod) * 1429) % mod;
        }
    };


    class BroadphaseMbp
    {
    private:

        std::unordered_map< vector3i, BroadphaseMbpRegion, BroadphaseHash > m_regions;
        std::vector< BroadphaseSimplex* > idToSimplex;
        vector3f m_cellsize;
        vector3f m_updatedRegions;

    public:

        BroadphaseMbp(const vector3f& cellsize) :
        m_cellsize(cellsize),
        idToSimplex(MAX_OBJECTS, nullptr)
        {}

        const BroadphaseMbpRegion& operator[](vector3i& idx)
        {
            return m_regions[idx];
        }

        const bool count(vector3i& idx) const;

        const BroadphaseMbpRegion& getRegionFromPosition(vector3f& pos)
        {
            vector3i region = {(pos.x/m_cellsize.x), (pos.y/m_cellsize.y), (pos.z/m_cellsize.z)};
            return m_regions[region];
        }

        void addObject(CollisionObject* obj);

        void removeObject(CollisionObject* obj);

        ~BroadphaseMbp();
    };

}
