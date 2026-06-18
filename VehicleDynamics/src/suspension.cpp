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
            const double relativeVelocity = this->State.WheelMountMotion[wheelcnt].frame_velocity.translation.z - this->State.SuspensionMotion[wheelcnt].frame_velocity.translation.z * 0;
            damperForce = this->Spec.SuspensionDamper[wheelcnt].z * relativeVelocity;
            //std::cout << "wheel mount position : " << relativeVelocity<< std::endl;
        }

        this->State.SuspensionForce[wheelcnt].Force.z = this->Spec.SuspensionSpring[wheelcnt].z * displacement + damperForce + this->Spec.Mass * 9.80665 / this->Spec.TotalWheels;


        
        //std::cout << "suspension force : " << this->State.SuspensionForce[wheelcnt].Force.x << ", " << this->State.SuspensionForce[wheelcnt].Force.y << ", " << this->State.SuspensionForce[wheelcnt].Force.z << std::endl;
    }
}


