#include <iostream>
#include "collision.h"
#include "kinematics.h"
#include "linearalgebra.h"
#include "ODESolver.h"
#include "vehicle.h"
int main()
{
    //Terrain Terrain1;
    //Terrain1.loadPLY("/home/kwon/Downloads/add_point.ply");
    Vehicle testvehicle("./vehicle_example");


    std::cout << testvehicle.Spec.TotalWheels << std::endl;
    std::cout << testvehicle.Spec.Mass << std::endl;
    if (testvehicle.Spec.SuspensionPosition != nullptr)
        std::cout << testvehicle.State.WheelMotion[3].frame_velocity.frame_position.translation.z << std::endl;
    else{std::cout << "null" << std::endl;}
    //Position VehiclePosition{Vec3{0,0,3},Vec3{0,0,0}};
    for(int ii = 0 ; ii < 20000 ; ii++)
    {
        testvehicle.Update();
        if(ii % 10 == 0)    std::cout << "vehicle pos : " << testvehicle.State.GlobalVehicleMotion.frame_velocity.frame_position.translation.x << ", " << testvehicle.State.GlobalVehicleMotion.frame_velocity.frame_position.translation.y << ", " << testvehicle.State.GlobalVehicleMotion.translation.z << ", " << testvehicle.State.GlobalVehicleMotion.frame_velocity.frame_position.orientation.x << ", " << testvehicle.State.GlobalVehicleMotion.frame_velocity.frame_position.orientation.y << ", " << testvehicle.State.GlobalVehicleMotion.orientation.z << std::endl;
        //if(ii % 100 == 0)     std::cout << "wheel pos : " << testvehicle.State.GlobalSuspensionMotion[3].frame_velocity.frame_position.translation.x << ", " << testvehicle.State.GlobalSuspensionMotion[2].frame_velocity.frame_position.translation.y << ", " << testvehicle.State.GlobalSuspensionMotion[].frame_velocity.frame_position.translation.z << std::endl;
        //std::cout << testvehicle.State.GlobalWheelMotion[3].frame_velocity.frame_position.translation.z << std::endl;
    }
    

    position Vehicle{"world", "vehicle", Vec3{-30, 100, 3}, Vec3{0, 0, 0}};
    


    position* GlobalWheelPosition = new position[4];
    position* GlobalWheelMountPosition = new position[4];


    for(int i = 0 ; i < 4 ; i++)
    {
        GlobalWheelMountPosition[i] = Vehicle.ForwardKinematics(testvehicle.Spec.WheelMount[i]);
        GlobalWheelPosition[i] = GlobalWheelMountPosition[i].ForwardKinematics(testvehicle.Spec.SuspensionPosition[i]);
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
    //Terrain1.getHeightAndNormal(GlobalWheelPosition[0].translation);
}
