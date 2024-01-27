#include "collision/broadphase.h"

namespace physics
{
    const long long BroadphaseHash::mod = 1ll << 32;

    #define DEFAULTLOC UINT_MAX

    void BroadphaseMbpRegion::addSimplex(BroadphaseSimplex* simp)
    {
        if (m_objLocations[simp->id] == DEFAULTLOC)
            return;

        m_objLocations[simp->id] = m_objects.size();
        m_objects.push_back(simp);
    }

    void BroadphaseMbpRegion::removeSimplex(ObjectHandle handle)
    {
        size_t pos = m_objLocations[handle];
        size_t rid = m_objects.back()->id;
        m_objects[pos] = m_objects.back();
        m_objLocations[handle] = DEFAULTLOC;
        m_objLocations[rid] = pos;
        m_objects.pop_back();
    }

    void BroadphaseMbpRegion::generatePairs()
    {
        std::vector<BroadphaseSimplex*> sapvector = m_objects;
        sap0(sapvector.begin(), sapvector.end(), sapvector.size(), 0);
    }

    void BroadphaseMbpRegion::sap0(std::vector<BroadphaseSimplex*>::iterator begin, 
    std::vector<BroadphaseSimplex*>::iterator end, size_t sz, int consecutiveNonSplits)
    {
        if (end - begin <= 1 || consecutiveNonSplits == 3)
        {
            for (auto it1 = begin; it1 != end; ++it1)
            {
                for (auto it2 = it1 + 1; it2 != end; ++it2)
                {
                    m_pairs[m_numPairs++] = new BroadphasePairs((*it1)->id, (*it2)->id);
                }
            }
            return;
        }

        std::sort(begin, end, BroadphaseSimplexCmp< 0 >());
        std::vector<BroadphaseSimplex*>::iterator l = begin + sz/2, r = l + 1;
        bool canSplit;
        std::vector<BroadphaseSimplex*>::iterator split;

        while (l != begin && r != end)
        {
            if ( (*(l-1))->aabb.m_min[ 0 ] < (*l)->aabb.m_max[ 0 ] )
            {
                canSplit = true;
                split = l;
                break;
            }

            if ( (*(r-1))->aabb.m_min[ 0 ] < (*r)->aabb.m_max[ 0 ] )
            {
                canSplit = true;
                split = r;
                break;
            }
        }
        
        if (canSplit)
        {
            sap1(begin, split, split - begin, 0);
            sap1(split, end, split - begin, 0);
        }
        else
        {
            sap1(begin, end, sz, consecutiveNonSplits + 1);
        }
    }

    void BroadphaseMbpRegion::sap1(std::vector<BroadphaseSimplex*>::iterator begin, 
    std::vector<BroadphaseSimplex*>::iterator end, size_t sz, int consecutiveNonSplits)
    {
        if (end - begin <= 1 || consecutiveNonSplits == 3)
        {
            for (auto it1 = begin; it1 != end; ++it1)
            {
                for (auto it2 = it1 + 1; it2 != end; ++it2)
                {
                    m_pairs[m_numPairs++] = new BroadphasePairs((*it1)->id, (*it2)->id);
                }
            }
            return;
        }

        std::sort(begin, end, BroadphaseSimplexCmp< 1 >());
        std::vector<BroadphaseSimplex*>::iterator l = begin + sz/2, r = l + 1;
        bool canSplit;
        std::vector<BroadphaseSimplex*>::iterator split;

        while (l != begin && r != end)
        {
            if ( (*(l-1))->aabb.m_min[ 1 ] < (*l)->aabb.m_max[ 1 ] )
            {
                canSplit = true;
                split = l;
                break;
            }

            if ( (*(r-1))->aabb.m_min[ 1 ] < (*r)->aabb.m_max[ 1 ] )
            {
                canSplit = true;
                split = r;
                break;
            }
        }
        
        if (canSplit)
        {
            sap2(begin, split, split - begin, 0);
            sap2(split, end, split - begin, 0);
        }
        else
        {
            sap2(begin, end, sz, consecutiveNonSplits + 1);
        }
    }

    void BroadphaseMbpRegion::sap2(std::vector<BroadphaseSimplex*>::iterator begin, 
    std::vector<BroadphaseSimplex*>::iterator end, size_t sz, int consecutiveNonSplits)
    {
        if (end - begin <= 1 || consecutiveNonSplits == 3)
        {
            for (auto it1 = begin; it1 != end; ++it1)
            {
                for (auto it2 = it1 + 1; it2 != end; ++it2)
                {
                    m_pairs[m_numPairs++] = new BroadphasePairs((*it1)->id, (*it2)->id);
                }
            }
            return;
        }

        std::sort(begin, end, BroadphaseSimplexCmp< 2 >());
        std::vector<BroadphaseSimplex*>::iterator l = begin + sz/2, r = l + 1;
        bool canSplit;
        std::vector<BroadphaseSimplex*>::iterator split;

        while (l != begin && r != end)
        {
            if ( (*(l-1))->aabb.m_min[ 2 ] < (*l)->aabb.m_max[ 2 ] )
            {
                canSplit = true;
                split = l;
                break;
            }

            if ( (*(r-1))->aabb.m_min[ 2 ] < (*r)->aabb.m_max[ 2 ] )
            {
                canSplit = true;
                split = r;
                break;
            }
        }
        
        if (canSplit)
        {
            sap0(begin, split, split - begin, 0);
            sap0(split, end, split - begin, 0);
        }
        else
        {
            sap0(begin, end, sz, consecutiveNonSplits + 1);
        }
    }

    #undef DEFAULTLOC

    // BroadphaseMbp

    const bool BroadphaseMbp::count(vector3i& idx) const
    {
        return m_regions.count(idx);
    }

    void BroadphaseMbp::addObject(BroadphaseSimplex* simp)
    {
        const AABB& aabb = simp->aabb;
        const vector3f &rmin = aabb.m_min, &rmax = aabb.m_max;
        idToSimplex[simp->id] = simp;
        for (int i = rmin.x, ie = rmax.x; i <= ie; ++i)
        {
            for (int j = rmin.y, je = rmax.y; j < je; ++j)
            {
                for (int k = rmin.z, ke = rmax.z; k < ke; ++k)
                {
                    if (m_regions[{i, j, k}])
                        m_regions[{i, j, k}] = new BroadphaseMbpRegion(m_pairs, m_numPairs);
                    m_regions[{i, j, k}]->addSimplex(simp);
                }
            }
        }
    }

    void BroadphaseMbp::removeObject(ObjectHandle handle)
    {
        BroadphaseSimplex* simp = idToSimplex[handle];
        idToSimplex[handle] = nullptr;

        vector3f rmin = simp->aabb.m_min, rmax = simp->aabb.m_max;

        for (int i = rmin.x, ie = rmax.x; i <= ie; ++i)
        {
            for (int j = rmin.y, je = rmax.y; j < je; ++j)
            {
                for (int k = rmin.z, ke = rmax.z; k < ke; ++k)
                    m_regions[{i, j, k}]->removeSimplex(handle);
            }
        }

        delete simp;
    }

    void BroadphaseMbp::batchAdd()
    {
        while (m_numObjectsToAdd)
        {
            addObject(m_objectsToAdd[--m_numObjectsToAdd]);
        }
    }

    void BroadphaseMbp::batchRemove()
    {
        while (m_numObjectsToRemove)
        {
            removeObject(m_objectsToRemove[--m_numObjectsToRemove]);
        }
    }

    BroadphaseMbp::~BroadphaseMbp()
    {
        for (BroadphaseSimplex* simp : idToSimplex)
        {
            delete simp;
        }

        while (m_numObjectsToAdd)
        {
            delete m_objectsToAdd[--m_numObjectsToAdd];
        }
    }
}
