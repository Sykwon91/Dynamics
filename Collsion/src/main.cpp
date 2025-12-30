#include <iostream>
#include "collision.h"

int main()
{
    Position center{};
    center.set(0, 0, 0, 0, 0, 0);

    Cylinder cyl(1.01, 0.5, center);
    Box box(1.0, 0.99, 1.0, center);
    Plane plane({0,0,1},{0,0,-1});

    Position p{};
    p.set(0.0, 0.0, -0.5, 0, 0, 0);
    Vec3 test = plane.collisionPoint(cyl);
    cyl.checkcollsion(test);
    box.checkcollsion(p.translation);
    

    std::cout << "cyl collision = "
              << (cyl.collision ? "true" : "false")
              << std::endl;

    std::cout << "box collision = "
              << (box.collision ? "true" : "false")
              << std::endl;
    std::cout << test.x << std::endl;
    std::cout << test.y << std::endl;
    std::cout << test.z << std::endl;
}
