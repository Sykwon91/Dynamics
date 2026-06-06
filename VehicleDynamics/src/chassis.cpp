#include "vehicle.h"


void Vehicle::ChassisDynamics()
{
    this->State.BodyForce = ApplyForce(this->State.GravityBodyForce,this->Spec.CenterOfMass) ;
    for(int wheelcnt = 0 ; wheelcnt < this->Spec.TotalWheels ; wheelcnt++)
    {
        this->State.BodyForce += ApplyForce(this->State.SuspensionForce[wheelcnt], this->State.SuspensionMotion[wheelcnt].frame_velocity.frame_position.translation);
        
    }


    this->State.LocalVehicleMotion.translation = this->State.BodyForce.Force/this->Spec.Mass ;
    this->State.LocalVehicleMotion.orientation = Vec3{this->State.BodyForce.Torque.x/this->Spec.Inertia.x, this->State.BodyForce.Torque.y/this->Spec.Inertia.y, this->State.BodyForce.Torque.z/this->Spec.Inertia.z};
    //std::cout << "chassis acc : " << this->State.LocalVehicleMotion.translation.x << ", " << this->State.LocalVehicleMotion.translation.y << ", " << this->State.LocalVehicleMotion.translation.z << std::endl;
}



void Vehicle::BodyGravity()
{
    Vec3 Gravity{0,0,-9.80665};
    this->State.GravityBodyForce.Force = this->State.GlobalVehicleMotion.frame_velocity.frame_position.orientation.toRotation() * (this->Spec.Mass * Gravity);


    //std::cout << "gravity :"
    //<<  this->State.GravityBodyForce.Force.x << ", "
    //<<  this->State.GravityBodyForce.Force.y << ", "
    //<<  this->State.GravityBodyForce.Force.z 
    //<< std::endl;
}