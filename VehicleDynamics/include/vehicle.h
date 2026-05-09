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
            motion* ContactPoint = nullptr;
            Wrench* ContactWrench = nullptr;
            motion* WheelMountMotion = nullptr;
            Wrench* WheelMountForce = nullptr;
            motion* WheelMotion = nullptr;
            Wrench* WheelForce = nullptr;
            motion* SuspensionMotion = nullptr;
            Wrench* SuspensionForce = nullptr;
            motion LocalVehicleMotion;
            motion* GlobalWheelMotion = nullptr;
            motion* GlobalWheelMountMotion = nullptr;
            motion* GlobalSuspensionMotion = nullptr;
            motion GlobalVehicleMotion;
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
