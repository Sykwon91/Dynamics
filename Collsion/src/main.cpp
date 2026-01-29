#include <iostream>
#include "collision.h"

int main()
{
    Position center{};
    center.set(0, 0, 0, 0, 0.1, 0);

    Cylinder cyl(1.01, 0.5, center);
    Box box(2.0, 1, 1.0, center);
    Plane plane({0,0,1},{0,0,0});

    Position p{};
    p.set(0.0, 0.0, -0.5, 0, 0, 0);
    Vec3 test = plane.closest(cyl);
    Vec3 tests = plane.closest(box);
    cyl.checkcollsion(test);
    box.checkcollsion(p.translation);
    

    std::cout << "cyl collision = "
              << (cyl.collision ? "true" : "false")
              << std::endl;

    std::cout << tests.x << std::endl;
    std::cout << tests.y << std::endl;
    std::cout << tests.z << std::endl;
}
