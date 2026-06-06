#include "vehicle.h"

void Vehicle::WheelMountDynamcis()
{
    for(int wheelcnt = 0 ; wheelcnt < this->Spec.TotalWheels ; wheelcnt++)
    {
        if (this->Spec.WheelSpring == nullptr || this->State.WheelForce == nullptr)
            continue;

        const double displacement = this->State.ContactPoint[wheelcnt].translation.z - this->State.WheelMountMotion[wheelcnt].frame_velocity.frame_position.translation.z;
        double damperForce = 0.0;
        if (this->Spec.WheelDamper != nullptr)
        {
            const double relativeVelocity = - this->State.WheelMountMotion[wheelcnt].frame_velocity.translation.z;
            damperForce = this->Spec.WheelDamper[wheelcnt].z * relativeVelocity;
        }

        this->State.WheelForce[wheelcnt].Force.z = this->Spec.WheelSpring[wheelcnt].z * displacement + damperForce + this->Spec.WheelMass * 9.80665  + this->Spec.Mass * 9.80665 / this->Spec.TotalWheels;
        this->State.WheelMountMotion[wheelcnt].translation.z = (this->State.WheelForce[wheelcnt].Force.z  - this->State.SuspensionForce[wheelcnt].Force.z ) / this->Spec.WheelMass;
    }
    //std::cout << "wheel Mount Motion acc : " << this->State.WheelMountMotion[0].frame_acceleration.translation.x << ", " << this->State.WheelMountMotion[0].frame_acceleration.translation.y << ", " << this->State.WheelMountMotion[0].frame_acceleration.translation.z << std::endl;

}

void Vehicle::WheelGravity()
{
    Vec3 Gravity{0,0,-9.80665};
    for(int wheelcnt = 0 ; wheelcnt < this->Spec.TotalWheels ; wheelcnt++)
    {
        this->State.GravityWheelForce[wheelcnt].Force = this->State.WheelMountMotion[wheelcnt].frame_velocity.frame_position.orientation.toRotation() * (this->Spec.WheelMass * Gravity);
    }
    
}