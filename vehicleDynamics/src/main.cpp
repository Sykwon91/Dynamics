#include <iostream>
#include "collision.h"
#include "kinematics.h"
#include "linearalgebra.h"

int main()
{
    position* WheelPosition = new position[4];
    WheelPosition[0] = {"",Vec3{1,1,0},Vec3{0,0,0}};
    WheelPosition[1] = {"",Vec3{1,-1,0},Vec3{0,0,0}};
    WheelPosition[2] = {"",Vec3{-1,1,0},Vec3{0,0,0}};
    WheelPosition[3] = {"",Vec3{-1,-1,0},Vec3{0,0,0}};

    Position VehiclePosition{Vec3{1,0,3},Vec3{0,0,0}};

    position Vehicle{"",VehiclePosition.translation,VehiclePosition.rotation};


    position* GlobalWheelPosition = new position[4];


    for(int i = 0 ; i < 4 ; i++)
    {
        GlobalWheelPosition[i] = Vehicle.ForwardKinematics(WheelPosition[i]);
    }
    

    Cylinder Wheel[4] = {
                            Cylinder(0.3,0.2,{GlobalWheelPosition[0].translation,GlobalWheelPosition[0].orientation}),
                            Cylinder(0.3,0.2,{GlobalWheelPosition[1].translation,GlobalWheelPosition[1].orientation}),
                            Cylinder(0.3,0.2,{GlobalWheelPosition[2].translation,GlobalWheelPosition[2].orientation}),
                            Cylinder(0.3,0.2,{GlobalWheelPosition[3].translation,GlobalWheelPosition[3].orientation})
                        };
    
    Plane plane(Vec3{0,0,1},Vec3{0,0,0});


    std::cout << plane.closest(Wheel[0]).x << std::endl;
    std::cout << plane.closest(Wheel[0]).y << std::endl;
    std::cout << plane.closest(Wheel[0]).z << std::endl;

    Vec3 test = plane.closest(Wheel[0]);

    std::cout <<  GlobalWheelPosition[0].InverseKinematics(position{"", test, Vec3{}} ).translation.x << std::endl;
    std::cout <<  GlobalWheelPosition[0].InverseKinematics(position{"", test, Vec3{}} ).translation.y << std::endl;
    std::cout <<  GlobalWheelPosition[0].InverseKinematics(position{"", test, Vec3{}} ).translation.z << std::endl;

}
