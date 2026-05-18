#include "vehicle.h"

void Vehicle::BodyGravity()
{
    Vec3 Gravity{0,0,-9.80665};
    this->State.GravityBodyForce.Force = this->State.GlobalVehicleMotion.frame_position.orientation.toRotation() * (this->Spec.Mass * Gravity);


    //std::cout << "gravity :"
    //<<  this->State.GravityBodyForce.Force.x << ", "
    //<<  this->State.GravityBodyForce.Force.y << ", "
    //<<  this->State.GravityBodyForce.Force.z 
    //<< std::endl;
}


Wrench Vehicle::ApplyForce(Wrench Force, Vec3 Point)
{
    
    return {Force.Force ,Force.Torque + Point.cross(Force.Force)};
}