#include "vehicle.h"



Wrench Vehicle::ApplyForce(Wrench Force, Vec3 Point)
{
    
    return {Force.Force ,Force.Torque + Point.cross(Force.Force)};
}



void Vehicle::Update()
{
    
    
    
    this->State.LocalVehicleMotion = this->State.GlobalVehicleMotion.InverseKinematics(this->State.GlobalVehicleMotion);
    
    

    this->BodyGravity();
    this->WheelGravity();
    
    
    for(int wheelcnt = 0 ; wheelcnt < this->Spec.TotalWheels ; wheelcnt++)
    {

        this->State.ContactPoint[wheelcnt] = this->State.GlobalVehicleMotion.frame_velocity.frame_position.InverseKinematics(this->State.GlobalContactPoint[wheelcnt]);
        //this->State.GlobalWheelMountMotion[wheelcnt].frame_velocity.frame_position.translation.z = -1;
        this->State.SuspensionMotion[wheelcnt] = this->State.GlobalVehicleMotion.InverseKinematics(this->State.GlobalSuspensionMotion[wheelcnt]);
        this->State.WheelMountMotion[wheelcnt] = this->State.GlobalVehicleMotion.InverseKinematics(this->State.GlobalWheelMountMotion[wheelcnt]);
    }
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
    
    
    this->State.GlobalVehicleMotion.translation.z = 0;
    this->State.GlobalVehicleMotion = this->State.GlobalVehicleMotion.ForwardKinematics(this->State.LocalVehicleMotion);
    for(int wheelcnt = 0 ; wheelcnt < this->Spec.TotalWheels ; wheelcnt++)
    {
        this->State.GlobalSuspensionMotion[wheelcnt] = this->State.GlobalVehicleMotion.ForwardKinematics(this->State.SuspensionMotion[wheelcnt]);
        this->State.GlobalWheelMountMotion[wheelcnt] = this->State.GlobalVehicleMotion.ForwardKinematics(this->State.WheelMountMotion[wheelcnt]);
        
    }

    // Example: update wheel mount position based on vehicle position
    // Vehicle update logic can be implemented here.
}