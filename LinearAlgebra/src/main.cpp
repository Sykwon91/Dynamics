#include "linearalgebra.h"
#include <iostream>

int main()
{
    Mat3 test;
    Vec3 vec{1,0,0};
    test.Identity();
    std::cout << (test * vec).x <<std::endl;
    return 0;
}