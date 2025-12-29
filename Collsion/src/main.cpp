#include <iostream>
#include "collision.h"

int main()
{
    Position center{};
    center.set(0, 0, 0, -1.5707, 0, 0);

    Cylinder cyl(1.0, 0.2, center);
    Box box(1.0, 0.9, 1.0, center);

    Position p{};
    p.set(0.0, 0.0, -0.5, 0, 0, 0);

    cyl.checkcollsion(p);
    box.checkcollsion(p);

    std::cout << "cyl collision = "
              << (cyl.collision ? "true" : "false")
              << std::endl;

    std::cout << "box collision = "
              << (box.collision ? "true" : "false")
              << std::endl;            
}
