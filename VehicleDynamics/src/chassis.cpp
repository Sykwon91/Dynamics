#include "vehicle.h"


void Vehicle::ChassisDynamics()
{
    this->State.BodyForce = ApplyForce(this->State.GravityBodyForce,this->Spec.CenterOfMass);
    for(int wheelcnt = 0 ; wheelcnt < this->Spec.TotalWheels ; wheelcnt++)
    {
        this->State.BodyForce += ApplyForce(this->State.SuspensionForce[wheelcnt],this->State.SuspensionMotion[wheelcnt].frame_position.translation);
    }


    this->State.LocalVehicleMotion.frame_acceleration.translation = this->State.BodyForce.Force/this->Spec.Mass;

    //std::cout << this->State.BodyForce.Force.x << ", "
    //<< this->State.BodyForce.Force.y << ", "
    //<< this->State.BodyForce.Force.z << ", "
    //<< this->State.BodyForce.Torque.x << ", "
    //<< this->State.BodyForce.Torque.y << ", "
    //<< this->State.BodyForce.Torque.z << ", "
    //<< std::endl;
}