#include "linearalgebra.h"
#include <iostream>

int main()
{
    Mat3 test;
    Vec3 vec{1,0,0};
    test.Rz(1.5708);
    std::cout << test.mat[2][2] <<std::endl;
    std::cout << (test.toEuler()).y <<std::endl;
    return 0;
}