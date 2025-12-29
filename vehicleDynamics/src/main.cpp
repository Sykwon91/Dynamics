#include <iostream>
#include "collision.h"

int main()
{
    Position center{};
    center.set(0, 0, 0, 1.5707, 0, 0);

    Cylinder cyl(1.0, 0.2, center);

    Position p{};
    p.set(0.0, 0.0, 0.2, 0, 0, 0);

    cyl.checkcollsion(p);

    std::cout << "collision = "
              << (cyl.collision ? "true" : "false")
              << std::endl;
}
