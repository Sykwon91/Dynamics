#include <iostream>
#include "collision.h"
#include "kinematics.h"
#include "linearalgebra.h"
#include "TerrainMechanics.h"

int main()
{
    Terrain Terrain1;
    Terrain1.loadPLY("/home/kwon/Downloads/add_point.ply");
    position* WheelMount = new position[4];
    WheelMount[0] = {"vehicle", "wheel_fl_mount", Vec3{1, 1, -0.3}, Vec3{0, 0, 0}};
    WheelMount[1] = {"vehicle", "wheel_fr_mount", Vec3{1, -1, -0.3}, Vec3{0, 0, 0}};
    WheelMount[2] = {"vehicle", "wheel_rl_mount", Vec3{-1, 1, -0.3}, Vec3{0, 0, 0}};
    WheelMount[3] = {"vehicle", "wheel_rr_mount", Vec3{-1, -1, -0.3}, Vec3{0, 0, 0}};
    position* WheelPosition = new position[4];
    WheelPosition[0] = {"wheel_fl_mount", "wheel_fl", Vec3{0, 0, 0}, Vec3{0, 0, 0}};
    WheelPosition[1] = {"wheel_fr_mount", "wheel_fr", Vec3{0, 0, 0}, Vec3{0, 0, 0}};
    WheelPosition[2] = {"wheel_rl_mount", "wheel_rl", Vec3{0, 0, 0}, Vec3{0, 0, 0}};
    WheelPosition[3] = {"wheel_rr_mount", "wheel_rr", Vec3{0, 0, 0}, Vec3{0, 0, 0}};

    //Position VehiclePosition{Vec3{0,0,3},Vec3{0,0,0}};

    position Vehicle{"world", "vehicle", Vec3{-30, 300, 3}, Vec3{0, 0, 0}};
    


    position* GlobalWheelPosition = new position[4];
    position* GlobalWheelMountPosition = new position[4];


    for(int i = 0 ; i < 4 ; i++)
    {
        GlobalWheelMountPosition[i] = Vehicle.ForwardKinematics(WheelMount[i]);
        GlobalWheelPosition[i] = GlobalWheelMountPosition[i].ForwardKinematics(WheelPosition[i]);
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

    std::cout <<  GlobalWheelPosition[0].translation.x << std::endl;
    std::cout <<  GlobalWheelPosition[0].translation.y << std::endl;
    std::cout <<  GlobalWheelPosition[0].translation.z << std::endl;
    Terrain1.getHeightAndNormal(GlobalWheelPosition[0].translation);
}
