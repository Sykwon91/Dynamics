#include "missile.h"

#include "raylib.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{
struct CameraControl
{
    double sideOffset = 0.0;
    double upOffset = 12.0;
    double followDistance = 55.0;
};

struct ManualControl
{
    Vec3 commandBody{0.0, 0.0, 0.0};
};

Vector3 toRaylib(const Vec3& value)
{
    return Vector3{
        static_cast<float>(value.x),
        static_cast<float>(value.z),
        static_cast<float>(value.y)
    };
}

double radToDeg(double value)
{
    return value * 180.0 / 3.14159265358979323846;
}

Vec3 makeTargetVelocity(double time)
{
    return Vec3{74.0 * std::sin(time * 1.35), 138.0, 0.0};
}

Vec3 makeLaunchVelocity()
{
    return Vec3{0.0, 0.0, 0.0};
}

Vec3 makeInitialTargetPosition()
{
    return Vec3{2600.0, 80.0, 0.2};
}

const char* phaseName(MissileFlightPhase phase)
{
    switch (phase)
    {
    case MissileFlightPhase::Popup:
        return "popup";
    case MissileFlightPhase::HighCruise:
        return "high cruise";
    case MissileFlightPhase::TerminalDive:
        return "terminal dive";
    case MissileFlightPhase::DirectIntercept:
        return "direct intercept";
    }

    return "unknown";
}

void drawTrajectory(const std::vector<Vec3>& samples, Color color)
{
    if (samples.size() < 2) return;

    for (std::size_t i = 1; i < samples.size(); ++i)
        DrawLine3D(toRaylib(samples[i - 1]), toRaylib(samples[i]), color);
}

void drawMissile(const Missile& missile)
{
    const Vec3 position = missile.GetPosition();
    const Vec3 attitude = missile.GetState().Motion.frame_velocity.frame_position.orientation;
    const Mat3 rotation = attitude.toRotation();
    const Vec3 forward = rotation * Vec3{1.0, 0.0, 0.0};
    const Vec3 right = rotation * Vec3{0.0, 1.0, 0.0};
    const Vec3 up = rotation * Vec3{0.0, 0.0, 1.0};

    const Vector3 center = toRaylib(position);
    const Vector3 nose = toRaylib(position + forward * 8.0);
    const Vector3 tail = toRaylib(position - forward * 3.0);

    DrawCylinderEx(tail, nose, 0.8f, 0.25f, 16, RED);
    DrawSphere(nose, 1.1f, ORANGE);
    DrawLine3D(center, toRaylib(position + missile.GetAcceleration() * 0.2), PURPLE);
    DrawLine3D(center, toRaylib(position + forward * 18.0), RED);
    DrawLine3D(center, toRaylib(position + right * 12.0), GREEN);
    DrawLine3D(center, toRaylib(position + up * 12.0), BLUE);
}

void handleCameraInput(CameraControl& control)
{
    const double dt = static_cast<double>(GetFrameTime());
    const double offsetSpeed = 45.0;

    if (IsKeyDown(KEY_A)) control.sideOffset -= offsetSpeed * dt;
    if (IsKeyDown(KEY_D)) control.sideOffset += offsetSpeed * dt;
    if (IsKeyDown(KEY_W)) control.upOffset += offsetSpeed * dt;
    if (IsKeyDown(KEY_S)) control.upOffset -= offsetSpeed * dt;
    if (IsKeyPressed(KEY_C)) control = CameraControl{};

    control.sideOffset = std::clamp(control.sideOffset, -80.0, 80.0);
    control.upOffset = std::clamp(control.upOffset, -20.0, 80.0);
}

void handleManualInput(ManualControl& control, double commandAcceleration)
{
    control.commandBody = Vec3{0.0, 0.0, 0.0};

    if (IsKeyDown(KEY_LEFT)) control.commandBody.y -= commandAcceleration;
    if (IsKeyDown(KEY_RIGHT)) control.commandBody.y += commandAcceleration;
    if (IsKeyDown(KEY_UP)) control.commandBody.z += commandAcceleration;
    if (IsKeyDown(KEY_DOWN)) control.commandBody.z -= commandAcceleration;
}

void updateMissileCamera(Camera3D& camera, const Missile& missile, const CameraControl& control)
{
    const Vec3 position = missile.GetPosition();
    const Vec3 attitude = missile.GetState().Motion.frame_velocity.frame_position.orientation;
    const Mat3 rotation = attitude.toRotation();
    const Vec3 forward = rotation * Vec3{1.0, 0.0, 0.0};
    const Vec3 right = rotation * Vec3{0.0, 1.0, 0.0};
    const Vec3 up = rotation * Vec3{0.0, 0.0, 1.0};

    const Vec3 cameraPosition = position - forward * control.followDistance
                               + right * control.sideOffset
                               + up * control.upOffset;
    const Vec3 cameraTarget = position + forward * 90.0 + right * (control.sideOffset * 0.15);

    camera.position = toRaylib(cameraPosition);
    camera.target = toRaylib(cameraTarget);
    camera.up = toRaylib(up);
    camera.fovy = 55.0f;
    camera.projection = CAMERA_PERSPECTIVE;
}
}

int main(int argc, char** argv)
{
    MissileSpec spec;
    spec.Mass = 14.5;
    spec.Thrust = 30000.0;
    spec.BurnTime = 8.0;
    spec.TargetCruiseSpeed = 500.0;
    spec.SustainMotorUntilHit = true;
    spec.NavigationGain = 3.0;
    spec.MaxLateralAcceleration = 950.0;
    spec.PopupAltitude = 650.0;
    spec.DirectInterceptRange = 1000.0;
    spec.TerminalDiveRange = 720.0;
    spec.TerminalAimDepth = 1.0;
    spec.AltitudeGain = 0.9;
    spec.VerticalDamping = 1.8;
    spec.MinimumDiveAngle = 0.55;
    spec.HitRadius = 4.0;
    spec.ManualCommandAcceleration = 120.0;
    spec.EnableSamplingMPC = true;
    spec.MPCCommandAcceleration = 180.0;
    spec.MPCHorizonSteps = 140;
    spec.MPCStride = 8;
    spec.TimeStep = 0.001;

    Missile missile(spec);
    missile.SetInitialState(Vec3{0.0, 0.0, 1.4}, Vec3{0.0, 0.0, 0.0});
    
    TargetState target;
    target.Position = makeInitialTargetPosition();
    target.Velocity = makeTargetVelocity(0.0);

    if (argc > 1 && std::string(argv[1]) == "--headless")
    {
        double maxAltitude = missile.GetPosition().z;
        double maxSpeed = missile.GetVelocity().norm();
        double terminalStartAltitude = 0.0;
        double terminalStartRange = 0.0;
        double terminalDiveAngle = 0.0;
        double minTerminalSpeed = 1.0e12;
        double closestMissDistance = 1.0e12;
        double closestMissTime = 0.0;
        bool terminalSeen = false;

        while (!missile.GetState().TargetHit && missile.GetState().Time < 20.0)
        {
            target.Velocity = makeTargetVelocity(missile.GetState().Time);
            target.Position = target.Position + target.Velocity * spec.TimeStep;
            missile.Update(target);
            maxAltitude = std::max(maxAltitude, missile.GetPosition().z);
            maxSpeed = std::max(maxSpeed, missile.GetVelocity().norm());
            if (missile.GetState().FlightPhase == MissileFlightPhase::TerminalDive)
                minTerminalSpeed = std::min(minTerminalSpeed, missile.GetVelocity().norm());
            if (missile.GetState().MissDistance < closestMissDistance)
            {
                closestMissDistance = missile.GetState().MissDistance;
                closestMissTime = missile.GetState().Time;
            }

            if (!terminalSeen && missile.GetState().FlightPhase == MissileFlightPhase::TerminalDive)
            {
                terminalSeen = true;
                terminalStartAltitude = missile.GetPosition().z;
                const Vec3 relativePosition = target.Position - missile.GetPosition();
                terminalStartRange = std::sqrt(relativePosition.x * relativePosition.x
                                             + relativePosition.y * relativePosition.y);
                terminalDiveAngle = std::atan2(std::max(0.0, missile.GetPosition().z - target.Position.z),
                                               std::max(terminalStartRange, 1.0e-9));
            }
        }

        std::cout << "max altitude: " << maxAltitude << std::endl;
        std::cout << "max speed: " << maxSpeed << std::endl;
        std::cout << "terminal start altitude: " << terminalStartAltitude << std::endl;
        std::cout << "terminal start range: " << terminalStartRange << std::endl;
        std::cout << "terminal dive angle deg: " << terminalDiveAngle * 180.0 / 3.14159265358979323846 << std::endl;
        std::cout << "min terminal speed: " << minTerminalSpeed << std::endl;
        std::cout << "final speed: " << missile.GetVelocity().norm() << std::endl;
        std::cout << "final altitude: " << missile.GetPosition().z << std::endl;
        std::cout << "closest miss distance: " << closestMissDistance << std::endl;
        std::cout << "closest miss time: " << closestMissTime << std::endl;
        std::cout << "final miss distance: " << missile.GetState().MissDistance << std::endl;
        const Vec3 attitude = missile.GetState().Motion.frame_velocity.frame_position.orientation;
        std::cout << "final roll deg: " << radToDeg(attitude.x) << std::endl;
        std::cout << "final pitch deg: " << radToDeg(attitude.y) << std::endl;
        std::cout << "final yaw deg: " << radToDeg(attitude.z) << std::endl;
        std::cout << "target hit: " << missile.GetState().TargetHit << std::endl;
        return 0;
    }

    const int screenWidth = 1280;
    const int screenHeight = 720;
    InitWindow(screenWidth, screenHeight, "ATGM Missile Dynamics");
    SetTargetFPS(1000);

    Camera3D camera{};
    CameraControl cameraControl;
    ManualControl manualControl;
    updateMissileCamera(camera, missile, cameraControl);

    std::vector<Vec3> missilePath;
    std::vector<Vec3> targetPath;
    missilePath.reserve(4096);
    targetPath.reserve(4096);

    bool paused = false;

    while (!WindowShouldClose())
    {
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_R))
        {
            missile.SetInitialState(Vec3{0.0, 0.0, 1.4}, makeLaunchVelocity());
            manualControl = ManualControl{};
            target.Position = makeInitialTargetPosition();
            target.Velocity = makeTargetVelocity(0.0);
            missilePath.clear();
            targetPath.clear();
            paused = false;
        }
        handleCameraInput(cameraControl);
        handleManualInput(manualControl, spec.ManualCommandAcceleration);
        missile.SetManualCommandBody(manualControl.commandBody);

        if (!paused && !missile.GetState().TargetHit && missile.GetState().Time < 20.0)
        {
            for (int substep = 0; substep < 8 && !missile.GetState().TargetHit; ++substep)
            {
                target.Velocity = makeTargetVelocity(missile.GetState().Time);
                target.Position = target.Position + target.Velocity * spec.TimeStep;
                missile.Update(target);

                missilePath.push_back(missile.GetPosition());
                targetPath.push_back(target.Position);
            }
        }
        updateMissileCamera(camera, missile, cameraControl);

        BeginDrawing();
        ClearBackground(Color{235, 238, 241, 255});

        BeginMode3D(camera);
        DrawGrid(8000, 10.0f);
        DrawLine3D(Vector3{0.0f, 0.0f, 0.0f}, Vector3{4300.0f, 0.0f, 0.0f}, DARKGRAY);
        DrawLine3D(Vector3{0.0f, 0.0f, -300.0f}, Vector3{0.0f, 0.0f, 300.0f}, DARKGRAY);

        drawTrajectory(targetPath, Color{60, 130, 210, 255});
        drawTrajectory(missilePath, Color{220, 60, 45, 255});
        DrawSphere(toRaylib(target.Position), 4.0f, BLUE);
        DrawSphere(toRaylib(Vec3{target.Position.x, target.Position.y, target.Position.z + spec.PopupAltitude}), 2.5f, GOLD);
        DrawLine3D(toRaylib(target.Position), toRaylib(Vec3{target.Position.x, target.Position.y, target.Position.z + spec.PopupAltitude}), Color{90, 90, 90, 120});
        DrawCube(toRaylib(makeInitialTargetPosition()), 8.0f, 2.0f, 8.0f, Color{40, 90, 160, 120});
        drawMissile(missile);

        EndMode3D();

        const Vec3 p = missile.GetPosition();
        const Vec3 v = missile.GetVelocity();
        const Vec3 mpcCommand = missile.GetMPCCommandBody();
        const Vec3 attitude = missile.GetState().Motion.frame_velocity.frame_position.orientation;
        DrawText(TextFormat("time %.2f s", missile.GetState().Time), 20, 20, 20, DARKGRAY);
        DrawText(TextFormat("miss %.2f m", missile.GetState().MissDistance), 20, 46, 20, DARKGRAY);
        DrawText(TextFormat("pos %.1f %.1f %.1f", p.x, p.y, p.z), 20, 72, 20, DARKGRAY);
        DrawText(TextFormat("speed %.1f m/s", v.norm()), 20, 98, 20, DARKGRAY);
        DrawText(TextFormat("phase %s", phaseName(missile.GetState().FlightPhase)), 20, 124, 20, DARKGRAY);
        DrawText(TextFormat("roll %.1f deg", radToDeg(attitude.x)), 20, 150, 20, DARKGRAY);
        DrawText(TextFormat("pitch %.1f deg", radToDeg(attitude.y)), 20, 176, 20, DARKGRAY);
        DrawText(TextFormat("yaw %.1f deg", radToDeg(attitude.z)), 20, 202, 20, DARKGRAY);
        DrawText(TextFormat("cam side %.1f up %.1f", cameraControl.sideOffset, cameraControl.upOffset), 20, 228, 20, DARKGRAY);
        DrawText(TextFormat("cmd body y %.1f z %.1f", manualControl.commandBody.y, manualControl.commandBody.z), 20, 254, 20, DARKGRAY);
        DrawText(TextFormat("mpc body y %.1f z %.1f", mpcCommand.y, mpcCommand.z), 20, 280, 20, DARKGRAY);
        DrawText(paused ? "paused" : (missile.GetState().TargetHit ? "target hit" : "tracking"), 20, 306, 20, missile.GetState().TargetHit ? GREEN : MAROON);

        EndDrawing();
    }

    CloseWindow();

    std::cout << "final miss distance: " << missile.GetState().MissDistance << std::endl;
    std::cout << "target hit: " << missile.GetState().TargetHit << std::endl;
    return 0;
}
