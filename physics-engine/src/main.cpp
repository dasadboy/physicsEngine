#include "collision/collision-manager.h"
#include <iostream>

int main()
{
    physics::CapsuleCollider cap;
    physics::SphereCollider sph;
    physics::PlaneCollider pln;
    
    physics::BroadphaseMbp bp ({30, 30, 30});

    return 0;
}
