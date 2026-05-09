#include "vehicle.h"

void Vehicle::WheelMountDynamcis()
{
    for(int wheelcnt = 0 ; wheelcnt < this->Spec.TotalWheels ; wheelcnt++)
    {
        if (this->Spec.WheelSpring == nullptr || this->State.WheelForce == nullptr)
            continue;

        const double displacement = this->State.ContactPoint[wheelcnt].frame_position.translation.z - this->State.WheelMountMotion[wheelcnt].frame_position.translation.z;
        double damperForce = 0.0;
        if (this->Spec.WheelDamper != nullptr)
        {
            const double relativeVelocity = this->State.ContactPoint[wheelcnt].frame_velocity.translation.z - this->State.WheelMountMotion[wheelcnt].frame_velocity.translation.z;
            damperForce = this->Spec.WheelDamper[wheelcnt].z * relativeVelocity;
        }

        this->State.WheelForce[wheelcnt].Force.z = this->Spec.WheelSpring[wheelcnt].z * displacement + damperForce + this->Spec.WheelMass * 9.81  + this->Spec.Mass * 9.81 / this->Spec.TotalWheels;
    }
}