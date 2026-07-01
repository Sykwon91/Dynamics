#ifndef MISSILE_H
#define MISSILE_H

#include "kinematics.h"
#include "linearalgebra.h"

struct TargetState
{
    Vec3 Position{0.0, 0.0, 0.0};
    Vec3 Velocity{0.0, 0.0, 0.0};
};

enum class MissileFlightPhase
{
    Popup,
    HighCruise,
    TerminalDive,
    DirectIntercept
};

struct MissileSpec
{
    double Mass = 14.5;
    double ReferenceArea = 0.018;
    double DragCoefficient = 0.45;
    double AirDensity = 1.225;
    double Thrust = 20000.0;
    double BurnTime = 8.0;
    double TargetCruiseSpeed = 500.0;
    bool SustainMotorUntilHit = true;
    Vec3 Inertia{0.08, 1.2, 1.2};
    Vec3 AttitudeControlGain{24.0, 28.0, 28.0};
    Vec3 AttitudeDamping{8.0, 9.0, 9.0};
    double MaxAngularAcceleration = 12.0;
    double MaxLateralAcceleration = 650.0;
    double NavigationGain = 3.0;
    double PopupAltitude = 650.0;
    double DirectInterceptRange = 1000.0;
    double TerminalDiveRange = 720.0;
    double TerminalAimDepth = 0.8;
    double AltitudeGain = 0.9;
    double VerticalDamping = 1.8;
    double MinimumDiveAngle = 0.75;
    double HitRadius = 4.0;
    double ManualCommandAcceleration = 120.0;
    bool EnableSamplingMPC = true;
    double MPCCommandAcceleration = 180.0;
    int MPCHorizonSteps = 140;
    int MPCStride = 8;
    double Gravity = 9.81;
    double TimeStep = 0.001;
};

struct MissileState
{
    acceleration Motion;
    double Time = 0.0;
    bool MotorBurning = true;
    bool TargetHit = false;
    bool DirectIntercept = false;
    MissileFlightPhase FlightPhase = MissileFlightPhase::Popup;
    double MissDistance = 0.0;
};

class Missile
{
public:
    explicit Missile(const MissileSpec& spec = MissileSpec{});

    void SetInitialState(const Vec3& position, const Vec3& velocity);
    void SetManualCommandBody(const Vec3& command);
    void Update(const TargetState& target);

    const MissileSpec& GetSpec() const;
    const MissileState& GetState() const;
    Vec3 GetPosition() const;
    Vec3 GetVelocity() const;
    Vec3 GetAcceleration() const;
    Vec3 GetMPCCommandBody() const;

private:
    MissileSpec Spec;
    MissileState State;
    Vec3 ManualCommandBody{0.0, 0.0, 0.0};
    Vec3 MPCCommandBody{0.0, 0.0, 0.0};

    Vec3 BodyVectorToWorld(const Vec3& bodyVector) const;
    Vec3 ForwardDirection() const;
    Vec3 DragAcceleration() const;
    Vec3 ThrustAcceleration() const;
    Vec3 GuidanceAcceleration(const TargetState& target) const;
    Vec3 ManualCommandAcceleration() const;
    Vec3 SamplingMPCCommandBody(const TargetState& target) const;
    double EvaluateMPCCommand(const Vec3& commandBody, TargetState target) const;
    Vec3 AttitudeControlAcceleration(const TargetState& target) const;
    TargetState GuidanceTarget(const TargetState& target) const;
    void UpdateFlightPhase(const TargetState& target);
    void UpdateAttitudeDynamics(const TargetState& target);
};

#endif
