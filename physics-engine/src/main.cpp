#include "collision/collision-manager.h"
#include <iostream>

int main()
{
    physics::CollisionManager cm (200);

    physics::CollisionObject co (physics::ColliderType::SPHERE);

    physics::CollisionObject* co2 = new physics::CollisionObject(physics::ColliderType::SPHERE);

    cm.addCollisionObjects({co2});

    return 0;
}
