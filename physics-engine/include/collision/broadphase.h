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
        const ObjectHandle objA;
        const ObjectHandle objB;

        BroadphasePairs(ObjectHandle a, ObjectHandle b) :
        objA(a),
        objB(b)
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


    class BroadphaseMbpRegion
    {
        #define DEFAULTLOC UINT_MAX

        std::vector<BroadphaseSimplex*> m_objects;
        std::vector<unsigned int> m_objLocations;
        std::array<BroadphasePairs*, MAX_PAIRS>& m_pairs;
        size_t& m_numPairs;
    
    public:

        BroadphaseMbpRegion(std::array<BroadphasePairs*, MAX_PAIRS>& pairs, size_t& s) :
        m_objLocations(UINT_MAX, DEFAULTLOC),
        m_pairs(pairs),
        m_numPairs(s)
        {}

        void addSimplex(BroadphaseSimplex* simp);

        void removeSimplex(ObjectHandle handle);

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

        size_t operator()(const vector3ll& key) const
        {
            return ((key.x * 2467) % mod + (key.y * 71167) % mod + (key.z * 1429) % mod) % mod;
        }
    };

    class CollisionManager;

    class BroadphaseMbp
    {
    private:

        std::unordered_map< vector3ll, BroadphaseMbpRegion*, BroadphaseHash > m_regions;
        std::vector< BroadphaseSimplex* > idToSimplex;
        vector3f m_cellsize;
        vector3f m_updatedRegions;
        
        std::array<BroadphaseSimplex*, MAX_OBJECTS> m_objectsToAdd;
        size_t m_numObjectsToAdd;
        std::array<ObjectHandle, MAX_OBJECTS> m_objectsToRemove;
        size_t m_numObjectsToRemove;

        std::array<BroadphasePairs*, MAX_PAIRS > m_pairs;
        size_t m_numPairs;

        void addObject(BroadphaseSimplex* simp);

        void removeObject(ObjectHandle handle);

        friend class CollisionManager;

    public:

        BroadphaseMbp(const vector3f& cellsize) :
        m_cellsize(cellsize),
        idToSimplex(MAX_OBJECTS, nullptr),
        m_numObjectsToAdd(0),
        m_numObjectsToRemove(0),
        m_numPairs(0)
        {}

        void queueForRemoval(std::vector<ObjectHandle> handles)
        {
            for (ObjectHandle handle : handles)
            {
                m_objectsToRemove[m_numObjectsToRemove++] = handle;
            }
        }

        void queueForAddition(std::vector<CollisionObject*>& objs)
        {
            for (CollisionObject* obj : objs)
            {
                BroadphaseSimplex* simp = new BroadphaseSimplex(obj->getid(), obj->getAABB());
                m_objectsToAdd[m_numObjectsToAdd++] = simp;
            }
        }

        void batchAdd();

        void batchRemove();

        const BroadphaseMbpRegion* operator[](vector3i& idx)
        {
            return m_regions[idx];
        }

        const bool count(vector3i& idx) const;

        const BroadphaseMbpRegion* getRegionFromPosition(vector3f& pos)
        {
            vector3i region = {static_cast<int> (pos.x/m_cellsize.x), static_cast<int> (pos.y/m_cellsize.y), static_cast<int> (pos.z/m_cellsize.z)};
            return m_regions[region];
        }

        ~BroadphaseMbp();
    };

}
