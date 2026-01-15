#include "kinematics.h"
#include <iostream>


int main()
{
    position x{"world", Vec3{1,0,0}, Vec3{0,0,1.5708}};
    position y{"base_link", Vec3{1,0,0}, Vec3{0,0,1.5708}};
    position xy = x.ForwardKinematics(y);
    position inv = x.InvereseKinematics(xy);

    std::cout << xy.frame <<std::endl;
    std::cout << xy.translation.x <<std::endl;
    std::cout << xy.translation.y <<std::endl;
    std::cout << inv.frame <<std::endl;
    std::cout << inv.translation.x <<std::endl;
    std::cout << inv.orientation.z <<std::endl;

    return 0;
}