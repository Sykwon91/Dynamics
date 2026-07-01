#include "missile.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double kSmall = 1e-9;

double clampMagnitude(double value, double limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

Vec3 limitVector(const Vec3& value, double limit)
{
    const double mag = value.norm();
    if (mag < kSmall || mag <= limit) return value;
    return value * (limit / mag);
}
}

Missile::Missile(const MissileSpec& spec)
    : Spec(spec)
{
}

void Missile::SetInitialState(const Vec3& position, const Vec3& velocity)
{
    State = MissileState{};
    ManualCommandBody = Vec3{0.0, 0.0, 0.0};
    MPCCommandBody = Vec3{0.0, 0.0, 0.0};
    State.Motion.frame_velocity.frame_position.frame = "world";
    State.Motion.frame_velocity.frame_position.childframe = "missile";
    State.Motion.frame_velocity.frame_position.translation = position;
    State.Motion.frame_velocity.translation = velocity;
    if (velocity.norm() > kSmall)
    {
        const double horizontalSpeed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
        State.Motion.frame_velocity.frame_position.orientation = Vec3{
            0.0,
            std::atan2(-velocity.z, horizontalSpeed),
            std::atan2(velocity.y, velocity.x)
        };
    }
}

void Missile::SetManualCommandBody(const Vec3& command)
{
    ManualCommandBody = limitVector(command, Spec.ManualCommandAcceleration);
}

void Missile::Update(const TargetState& target)
{
    if (State.TargetHit) return;

    State.MissDistance = (target.Position - GetPosition()).norm();
    if (State.MissDistance <= Spec.HitRadius)
    {
        State.TargetHit = true;
        return;
    }

    const Vec3 gravity{0.0, 0.0, -Spec.Gravity};
    State.MotorBurning = Spec.SustainMotorUntilHit || State.Time <= Spec.BurnTime;
    UpdateFlightPhase(target);
    MPCCommandBody = Spec.EnableSamplingMPC ? SamplingMPCCommandBody(target) : Vec3{0.0, 0.0, 0.0};

    State.Motion.translation = gravity
                             + DragAcceleration()
                             + ThrustAcceleration()
                             + GuidanceAcceleration(GuidanceTarget(target))
                             + ManualCommandAcceleration();

    State.Motion.frame_velocity.translation =
        State.Motion.frame_velocity.translation + State.Motion.translation * Spec.TimeStep;
    State.Motion.frame_velocity.frame_position.translation =
        State.Motion.frame_velocity.frame_position.translation + State.Motion.frame_velocity.translation * Spec.TimeStep;

    UpdateAttitudeDynamics(GuidanceTarget(target));

    State.Time += Spec.TimeStep;
    State.MissDistance = (target.Position - GetPosition()).norm();
    State.TargetHit = State.TargetHit || State.MissDistance <= Spec.HitRadius;
}

const MissileSpec& Missile::GetSpec() const
{
    return Spec;
}

const MissileState& Missile::GetState() const
{
    return State;
}

Vec3 Missile::GetPosition() const
{
    return State.Motion.frame_velocity.frame_position.translation;
}

Vec3 Missile::GetVelocity() const
{
    return State.Motion.frame_velocity.translation;
}

Vec3 Missile::GetAcceleration() const
{
    return State.Motion.translation;
}

Vec3 Missile::GetMPCCommandBody() const
{
    return MPCCommandBody;
}

Vec3 Missile::BodyVectorToWorld(const Vec3& bodyVector) const
{
    position bodyOrigin;
    bodyOrigin.frame = "missile";
    bodyOrigin.childframe = "body_origin";
    bodyOrigin.translation = Vec3{0.0, 0.0, 0.0};
    bodyOrigin.orientation = Vec3{0.0, 0.0, 0.0};

    position bodyPoint;
    bodyPoint.frame = "missile";
    bodyPoint.childframe = "body_vector";
    bodyPoint.translation = bodyVector;
    bodyPoint.orientation = Vec3{0.0, 0.0, 0.0};

    const position worldOrigin =
        State.Motion.frame_velocity.frame_position.ForwardKinematics(bodyOrigin);
    const position worldPoint =
        State.Motion.frame_velocity.frame_position.ForwardKinematics(bodyPoint);

    return worldPoint.translation - worldOrigin.translation;
}

Vec3 Missile::ForwardDirection() const
{
    return BodyVectorToWorld(Vec3{1.0, 0.0, 0.0}).normalized(1e-12);
}

Vec3 Missile::DragAcceleration() const
{
    const Vec3 velocity = GetVelocity();
    const double speed = velocity.norm();
    if (speed < kSmall || Spec.Mass < kSmall) return Vec3{0.0, 0.0, 0.0};

    const double drag = 0.5 * Spec.AirDensity * speed * speed * Spec.DragCoefficient * Spec.ReferenceArea;
    return velocity.normalized(kSmall) * (-drag / Spec.Mass);
}

Vec3 Missile::ThrustAcceleration() const
{
    if (!State.MotorBurning || Spec.Mass < kSmall) return Vec3{0.0, 0.0, 0.0};

    const double speed = GetVelocity().norm();
    if (speed >= Spec.TargetCruiseSpeed) return Vec3{0.0, 0.0, 0.0};

    const double speedError = Spec.TargetCruiseSpeed - speed;
    const double throttle = std::min(1.0, speedError / 50.0);
    return ForwardDirection() * (Spec.Thrust * throttle / Spec.Mass);
}

Vec3 Missile::GuidanceAcceleration(const TargetState& target) const
{
    const Vec3 relativePosition = target.Position - GetPosition();
    const Vec3 relativeVelocity = target.Velocity - GetVelocity();
    const double range = relativePosition.norm();
    if (range < kSmall) return Vec3{0.0, 0.0, 0.0};

    const Vec3 lineOfSight = relativePosition / range;
    const double closingVelocity = std::max(0.0, -relativeVelocity.dot(lineOfSight));
    const Vec3 lineOfSightRate = relativePosition.cross(relativeVelocity) / std::max(range * range, kSmall);
    Vec3 command = lineOfSightRate.cross(lineOfSight) * (Spec.NavigationGain * closingVelocity);
    command.z += Spec.AltitudeGain * relativePosition.z + Spec.VerticalDamping * relativeVelocity.z;

    return limitVector(command, Spec.MaxLateralAcceleration);
}

Vec3 Missile::ManualCommandAcceleration() const
{
    return BodyVectorToWorld(ManualCommandBody + MPCCommandBody);
}

Vec3 Missile::SamplingMPCCommandBody(const TargetState& target) const
{
    const double u = Spec.MPCCommandAcceleration;
    const Vec3 candidates[] = {
        {0.0, 0.0, 0.0},
        {0.0,  u, 0.0},
        {0.0, -u, 0.0},
        {0.0, 0.0,  u},
        {0.0, 0.0, -u},
        {0.0,  u * 0.70710678118,  u * 0.70710678118},
        {0.0,  u * 0.70710678118, -u * 0.70710678118},
        {0.0, -u * 0.70710678118,  u * 0.70710678118},
        {0.0, -u * 0.70710678118, -u * 0.70710678118}
    };

    Vec3 bestCommand{0.0, 0.0, 0.0};
    double bestCost = std::numeric_limits<double>::infinity();

    for (const Vec3& command : candidates)
    {
        const double cost = EvaluateMPCCommand(command, target);
        if (cost < bestCost)
        {
            bestCost = cost;
            bestCommand = command;
        }
    }

    return bestCommand;
}

double Missile::EvaluateMPCCommand(const Vec3& commandBody, TargetState target) const
{
    Missile sim = *this;
    sim.Spec.EnableSamplingMPC = false;
    sim.ManualCommandBody = ManualCommandBody + commandBody;
    sim.MPCCommandBody = Vec3{0.0, 0.0, 0.0};

    double minMissDistance = std::numeric_limits<double>::infinity();
    double speedCost = 0.0;
    double altitudeCost = 0.0;
    double terminalAngleCost = 0.0;
    bool terminalSeen = false;

    const int stride = std::max(1, Spec.MPCStride);
    for (int step = 0; step < Spec.MPCHorizonSteps && !sim.State.TargetHit; ++step)
    {
        target.Position = target.Position + target.Velocity * sim.Spec.TimeStep;
        sim.Update(target);

        if (step % stride != 0) continue;

        const Vec3 relative = target.Position - sim.GetPosition();
        const double miss = relative.norm();
        minMissDistance = std::min(minMissDistance, miss);

        const double speedError = sim.GetVelocity().norm() - Spec.TargetCruiseSpeed;
        speedCost += speedError * speedError * 0.0003;

        const TargetState guided = sim.GuidanceTarget(target);
        const double altitudeError = sim.GetPosition().z - guided.Position.z;
        altitudeCost += altitudeError * altitudeError * 0.00004;

        if (sim.State.FlightPhase == MissileFlightPhase::TerminalDive)
        {
            terminalSeen = true;
            const double horizontal = std::sqrt(relative.x * relative.x + relative.y * relative.y);
            const double diveAngle = std::atan2(std::max(0.0, sim.GetPosition().z - target.Position.z),
                                                std::max(horizontal, kSmall));
            const double angleError = std::max(0.0, Spec.MinimumDiveAngle - diveAngle);
            terminalAngleCost += angleError * angleError * 5000.0;
        }
    }

    const Vec3 finalRelative = target.Position - sim.GetPosition();
    const double finalMiss = finalRelative.norm();
    const double commandCost = commandBody.dot(commandBody) * 0.0002;
    const double terminalBonus = terminalSeen ? 0.0 : 2500.0;
    const double hitBonus = sim.State.TargetHit ? -10000.0 : 0.0;

    return std::min(minMissDistance, finalMiss) * 2.0
         + finalMiss * 0.35
         + speedCost
         + altitudeCost
         + terminalAngleCost
         + commandCost
         + terminalBonus
         + hitBonus;
}

Vec3 Missile::AttitudeControlAcceleration(const TargetState& target) const
{
    position targetPose;
    targetPose.frame = "world";
    targetPose.childframe = "guidance_target";
    targetPose.translation = target.Position;
    targetPose.orientation = Vec3{0.0, 0.0, 0.0};

    const position targetInBody =
        State.Motion.frame_velocity.frame_position.InverseKinematics(targetPose);
    const Vec3 losBody = targetInBody.translation;
    const double horizontal = std::sqrt(losBody.x * losBody.x + losBody.y * losBody.y);

    Vec3 attitudeError{
        -State.Motion.frame_velocity.frame_position.orientation.x,
        std::atan2(-losBody.z, std::max(horizontal, kSmall)),
        std::atan2(losBody.y, losBody.x)
    };

    Vec3 angularAcceleration{
        Spec.AttitudeControlGain.x * attitudeError.x
            - Spec.AttitudeDamping.x * State.Motion.frame_velocity.orientation.x,
        Spec.AttitudeControlGain.y * attitudeError.y
            - Spec.AttitudeDamping.y * State.Motion.frame_velocity.orientation.y,
        Spec.AttitudeControlGain.z * attitudeError.z
            - Spec.AttitudeDamping.z * State.Motion.frame_velocity.orientation.z
    };

    angularAcceleration.x /= std::max(Spec.Inertia.x, kSmall);
    angularAcceleration.y /= std::max(Spec.Inertia.y, kSmall);
    angularAcceleration.z /= std::max(Spec.Inertia.z, kSmall);
    return limitVector(angularAcceleration, Spec.MaxAngularAcceleration);
}

TargetState Missile::GuidanceTarget(const TargetState& target) const
{
    TargetState guidedTarget = target;

    if (State.DirectIntercept)
    {
        return guidedTarget;
    }

    if (State.FlightPhase == MissileFlightPhase::Popup
        || State.FlightPhase == MissileFlightPhase::HighCruise)
    {
        guidedTarget.Position.z = target.Position.z + Spec.PopupAltitude;
        guidedTarget.Velocity.z = 0.0;
        return guidedTarget;
    }

    guidedTarget.Position.z = target.Position.z - Spec.TerminalAimDepth;
    return guidedTarget;
}

void Missile::UpdateFlightPhase(const TargetState& target)
{
    if (State.FlightPhase == MissileFlightPhase::TerminalDive
        || State.FlightPhase == MissileFlightPhase::DirectIntercept)
    {
        return;
    }

    const Vec3 relativePosition = target.Position - GetPosition();
    const double range = relativePosition.norm();
    const double horizontalRange = std::sqrt(relativePosition.x * relativePosition.x
                                          + relativePosition.y * relativePosition.y);

    if (State.FlightPhase == MissileFlightPhase::Popup
        && State.Time <= Spec.TimeStep
        && range <= Spec.DirectInterceptRange)
    {
        State.DirectIntercept = true;
        State.FlightPhase = MissileFlightPhase::DirectIntercept;
        return;
    }

    if (State.FlightPhase == MissileFlightPhase::Popup
        && GetPosition().z >= Spec.PopupAltitude * 0.95)
    {
        State.FlightPhase = MissileFlightPhase::HighCruise;
    }

    const double requiredDiveAngle = std::atan2(std::max(0.0, GetPosition().z - target.Position.z),
                                                std::max(horizontalRange, kSmall));
    if (horizontalRange <= Spec.TerminalDiveRange && requiredDiveAngle >= Spec.MinimumDiveAngle)
    {
        State.FlightPhase = MissileFlightPhase::TerminalDive;
    }
}

void Missile::UpdateAttitudeDynamics(const TargetState& target)
{
    State.Motion.frame_velocity.orientation = Vec3{
        State.Motion.frame_velocity.orientation.x,
        State.Motion.frame_velocity.orientation.y,
        State.Motion.frame_velocity.orientation.z
    };
    State.Motion.orientation = AttitudeControlAcceleration(target);
    State.Motion.frame_velocity.orientation =
        State.Motion.frame_velocity.orientation + State.Motion.orientation * Spec.TimeStep;
    State.Motion.frame_velocity.frame_position.orientation =
        State.Motion.frame_velocity.frame_position.orientation
        + State.Motion.frame_velocity.orientation * Spec.TimeStep;

    State.Motion.frame_velocity.orientation.x =
        clampMagnitude(State.Motion.frame_velocity.orientation.x, 20.0);
    State.Motion.frame_velocity.orientation.y =
        clampMagnitude(State.Motion.frame_velocity.orientation.y, 20.0);
    State.Motion.frame_velocity.orientation.z =
        clampMagnitude(State.Motion.frame_velocity.orientation.z, 20.0);
}
