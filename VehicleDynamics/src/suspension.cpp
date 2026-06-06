#include "vehicle.h"


void Vehicle::SuspensionDynamics()
{
    for(int wheelcnt = 0 ; wheelcnt < this->Spec.TotalWheels ; wheelcnt++)
    {
        if (this->Spec.SuspensionSpring == nullptr || this->State.SuspensionForce == nullptr)
            continue;

        const double displacement = this->State.WheelMountMotion[wheelcnt].frame_velocity.frame_position.translation.z - this->State.SuspensionMotion[wheelcnt].frame_velocity.frame_position.translation.z;
        double damperForce = 0.0;
        if (this->Spec.SuspensionDamper != nullptr)
        {
            const double relativeVelocity = this->State.WheelMountMotion[wheelcnt].frame_velocity.translation.z - this->State.SuspensionMotion[wheelcnt].frame_velocity.translation.z;
            damperForce = this->Spec.SuspensionDamper[wheelcnt].z * relativeVelocity;
        }

        this->State.SuspensionForce[wheelcnt].Force.z = this->Spec.SuspensionSpring[wheelcnt].z * displacement + damperForce + this->Spec.Mass * 9.80665 / this->Spec.TotalWheels;
    }
}


