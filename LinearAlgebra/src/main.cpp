#include "linearalgebra.h"
#include <iostream>

int main()
{
    Mat3 test;
    Vec3 vec{1,0,0};
    test.Ry(1.5708);
    std::cout << (test* test * vec).x <<std::endl;
    return 0;
}