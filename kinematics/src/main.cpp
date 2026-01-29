#include "kinematics.h"
#include <iostream>


int main()
{
    position x{"world", "world", Vec3{1,2,3}, Vec3{0,0,0}};
    position y{"base_link","base_link", Vec3{1,2,3}, Vec3{0,0,0}};
    position xy = x.ForwardKinematics(y);
    position inv = x.InverseKinematics(xy);

    velocity x_v{x,Vec3{1,2,3},Vec3{0,0,2.0}};
    velocity y_v{y,Vec3{4,5,6},Vec3{0,0,1.0}};
    velocity xy_v = x_v.ForwardKinematics(y_v);
    velocity inv_v = x_v.InverseKinematics(xy_v);


    acceleration a_x{x_v,Vec3{0,0,0},Vec3{0,0,1}};
    acceleration a_y{y_v,Vec3{10,20,30},Vec3{0,0,3}};

    acceleration a_xy = a_x.ForwardKinematics(a_y);
    acceleration inv_a_xy = a_x.InverseKinematics(a_xy);

    std::cout << a_xy.translation.x << std::endl;
    std::cout << a_xy.translation.y << std::endl;
    std::cout << a_xy.translation.z << std::endl;

    std::cout << a_xy.orientation.z << std::endl;
    std::cout << inv_a_xy.translation.x << std::endl;
    std::cout << inv_a_xy.translation.y << std::endl;
    std::cout << inv_a_xy.translation.z << std::endl;

    return 0;
}