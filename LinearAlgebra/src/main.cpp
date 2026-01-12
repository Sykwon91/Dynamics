#include "linearalgebra.h"
#include <iostream>

int main()
{
    Mat3 test;
    test.Identity();
    std::cout << test.Identity().mat[0][0] <<std::endl;
    return 0;
}