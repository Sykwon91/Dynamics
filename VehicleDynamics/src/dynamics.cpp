#include "vehicle.h"



Wrench Vehicle::ApplyForce(Wrench Force, Vec3 Point)
{
    
    return {Force.Force ,Force.Torque + Point.cross(Force.Force)};
}



void Vehicle::Update()
{
    
    
    acceleration VehiclePosition;

    VehiclePosition.frame_velocity.frame_position = this->State.GlobalVehicleMotion.frame_velocity.frame_position;

    this->State.LocalVehicleMotion = this->State.GlobalVehicleMotion.InverseKinematics(VehiclePosition);
    
    

    
    
    
    for(int wheelcnt = 0 ; wheelcnt < this->Spec.TotalWheels ; wheelcnt++)
    {
        //this->State.GlobalContactPoint[wheelcnt].translation.z = -0.299f;   
        this->State.ContactPoint[wheelcnt] = VehiclePosition.frame_velocity.frame_position.InverseKinematics(this->State.GlobalContactPoint[wheelcnt]);
        //this->State.GlobalWheelMountMotion[wheelcnt].frame_velocity.frame_position.translation.z = 0.f;
        this->State.SuspensionMotion[wheelcnt] = VehiclePosition.InverseKinematics(this->State.GlobalSuspensionMotion[wheelcnt]);
        this->State.WheelMountMotion[wheelcnt] = VehiclePosition.InverseKinematics(this->State.GlobalWheelMountMotion[wheelcnt]);
    }
    this->BodyGravity();
    this->WheelGravity();
    this->SuspensionDynamics();
    this->WheelMountDynamcis();
    this->ChassisDynamics();

    odeSolver.solve(this->State.LocalVehicleMotion);
    
    for(int wheelcnt = 0 ; wheelcnt < this->Spec.TotalWheels ; wheelcnt++)
    {
        odeSolver.solve( this->State.WheelMountMotion[wheelcnt]);
        odeSolver.solve( this->State.WheelMotion[wheelcnt]);
    }
    //std::cout << "vehicle pos : " << this->State.GlobalVehicleMotion.translation.x << ", " << this->State.GlobalVehicleMotion.translation.y << ", " << this->State.GlobalVehicleMotion.frame_velocity.frame_position.translation.z << std::endl;
    
    
    //this->State.GlobalVehicleMotion.translation.z = 0;
    this->State.GlobalVehicleMotion = VehiclePosition.ForwardKinematics(this->State.LocalVehicleMotion);
    for(int wheelcnt = 0 ; wheelcnt < this->Spec.TotalWheels ; wheelcnt++)
    {
        this->State.GlobalSuspensionMotion[wheelcnt] = this->State.GlobalVehicleMotion.ForwardKinematics(this->State.SuspensionMotion[wheelcnt]);
        this->State.GlobalWheelMountMotion[wheelcnt] = this->State.GlobalVehicleMotion.ForwardKinematics(this->State.WheelMountMotion[wheelcnt]);
    }

    // Example: update wheel mount position based on vehicle position
    // Vehicle update logic can be implemented here.
}