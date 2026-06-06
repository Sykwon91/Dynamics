#ifndef VEHICLE_H
#define VEHICLE_H
#include <string>
#include "linearalgebra.h"
#include "kinematics.h"
#include "collision.h"
#include "TerrainMechanics.h"
#include "ODESolver.h"
#include "wrench.h"

class Vehicle
{
    private:
        
        struct input
        {
            double Throttle;
            double Brake;
            double Steering;
        };
        struct state
        {
            position* ContactPoint = nullptr;
            Wrench* ContactWrench = nullptr;
            acceleration* WheelMountMotion = nullptr;
            Wrench* WheelMountForce = nullptr;
            acceleration* WheelMotion = nullptr;
            Wrench* WheelForce = nullptr;
            acceleration* SuspensionMotion = nullptr;
            Wrench* SuspensionForce = nullptr;
            acceleration LocalVehicleMotion;
            position* GlobalContactPoint = nullptr;
            acceleration* GlobalWheelMotion = nullptr;
            acceleration* GlobalWheelMountMotion = nullptr;
            acceleration* GlobalSuspensionMotion = nullptr;
            acceleration GlobalVehicleMotion;
            Wrench GravityBodyForce;
            Wrench* GravityWheelForce;
            Wrench BodyForce;
        };
        struct spec
        {
            double Mass = 0.0;
            Vec3 Inertia{0.0, 0.0, 0.0};
            Vec3 CenterOfMass{0.0, 0.0, 0.0};
            double WheelMass = 0.0;
            Vec3 WheelInertia{0.0, 0.0, 0.0};
            position* WheelMount = nullptr;
            position* SuspensionPosition = nullptr;
            Vec3* SuspensionSpring = nullptr;
            Vec3* SuspensionDamper = nullptr;
            Vec3* WheelSpring = nullptr;
            Vec3* WheelDamper = nullptr;
            Cylinder* Wheel = nullptr;
            Box* Chassis = nullptr;
            int TotalWheels = 0;
        };
        ODESolver odeSolver;
        void SuspensionDynamics();
        void WheelMountDynamcis();
        void ChassisDynamics();
        void BodyGravity();
        void WheelGravity();
        Wrench ApplyForce(Wrench Force, Vec3 Point);

    public:
        Vehicle(std::string filename);
        ~Vehicle();
        input Input;
        state State;
        spec Spec;
        void Update();
        bool Visualize(const std::string& filename) const;
};

#endif
