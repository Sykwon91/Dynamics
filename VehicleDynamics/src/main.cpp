#include "vehicle.h"

#include "raylib.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct CameraControl
{
    float yaw = 35.0f * static_cast<float>(kPi / 180.0);
    float pitch = 24.0f * static_cast<float>(kPi / 180.0);
    float distance = 10.0f;
};

Vector3 toRaylib(const Vec3& value)
{
    return Vector3{
        static_cast<float>(value.x),
        static_cast<float>(value.z),
        static_cast<float>(value.y)
    };
}

Vec3 fromRaylib(const Vector3& value)
{
    return Vec3{
        static_cast<double>(value.x),
        static_cast<double>(value.z),
        static_cast<double>(value.y)
    };
}

double radToDeg(double value)
{
    return value * 180.0 / kPi;
}

bool fileExists(const std::string& path)
{
    std::ifstream file(path);
    return file.good();
}

std::string defaultVehicleConfig()
{
    const std::vector<std::string> candidates{
        "VehicleDynamics/vehicle_example.yaml",
        "../VehicleDynamics/vehicle_example.yaml",
        "./vehicle_example.yaml",
        "./vehicle_example"
    };

    for (const std::string& candidate : candidates)
    {
        if (fileExists(candidate))
            return candidate;
    }

    return "VehicleDynamics/vehicle_example.yaml";
}

void handleCameraInput(CameraControl& control)
{
    const float dt = GetFrameTime();
    const float rotateSpeed = 1.6f;
    const float zoomSpeed = 8.0f;

    if (IsKeyDown(KEY_LEFT)) control.yaw -= rotateSpeed * dt;
    if (IsKeyDown(KEY_RIGHT)) control.yaw += rotateSpeed * dt;
    if (IsKeyDown(KEY_UP)) control.pitch += rotateSpeed * dt;
    if (IsKeyDown(KEY_DOWN)) control.pitch -= rotateSpeed * dt;
    if (IsKeyDown(KEY_Q)) control.distance += zoomSpeed * dt;
    if (IsKeyDown(KEY_E)) control.distance -= zoomSpeed * dt;
    if (IsKeyPressed(KEY_C)) control = CameraControl{};

    control.pitch = std::clamp(control.pitch, -1.1f, 1.1f);
    control.distance = std::clamp(control.distance, 4.0f, 40.0f);
}

void updateCamera(Camera3D& camera, const CameraControl& control, const Vec3& target)
{
    const Vec3 offset{
        static_cast<double>(std::cos(control.pitch) * std::cos(control.yaw) * control.distance),
        static_cast<double>(std::cos(control.pitch) * std::sin(control.yaw) * control.distance),
        static_cast<double>(std::sin(control.pitch) * control.distance)
    };

    camera.position = toRaylib(target + offset);
    camera.target = toRaylib(target);
    camera.up = Vector3{0.0f, 1.0f, 0.0f};
    camera.fovy = 55.0f;
    camera.projection = CAMERA_PERSPECTIVE;
}

void drawBasis(const Vec3& origin, const Vec3& attitude, double scale)
{
    const Mat3 rotation = attitude.toRotation();
    const Vec3 forward = rotation * Vec3{1.0, 0.0, 0.0};
    const Vec3 right = rotation * Vec3{0.0, 1.0, 0.0};
    const Vec3 up = rotation * Vec3{0.0, 0.0, 1.0};

    DrawLine3D(toRaylib(origin), toRaylib(origin + forward * scale), RED);
    DrawLine3D(toRaylib(origin), toRaylib(origin + right * scale), GREEN);
    DrawLine3D(toRaylib(origin), toRaylib(origin + up * scale), BLUE);
}

void drawOrientedBox(const Vec3& origin, const Vec3& attitude, const Vec3& center, const Vec3& size, Color color)
{
    const Mat3 rotation = attitude.toRotation();
    const Vec3 half = size * 0.5;
    Vec3 corners[8];
    int index = 0;

    for (double sx : {-1.0, 1.0})
    {
        for (double sy : {-1.0, 1.0})
        {
            for (double sz : {-1.0, 1.0})
            {
                const Vec3 local{
                    center.x + sx * half.x,
                    center.y + sy * half.y,
                    center.z + sz * half.z
                };
                corners[index++] = origin + rotation * local;
            }
        }
    }

    const int edges[][2]{
        {0, 1}, {0, 2}, {0, 4}, {3, 1}, {3, 2}, {3, 7},
        {5, 1}, {5, 4}, {5, 7}, {6, 2}, {6, 4}, {6, 7}
    };

    for (const auto& edge : edges)
        DrawLine3D(toRaylib(corners[edge[0]]), toRaylib(corners[edge[1]]), color);
}

void drawWheel(const acceleration& wheelMotion, double radius, double width)
{
    const Vec3 center = wheelMotion.frame_velocity.frame_position.translation;
    const Vec3 attitude = wheelMotion.frame_velocity.frame_position.orientation;
    const Mat3 rotation = attitude.toRotation();
    const Vec3 axle = rotation * Vec3{0.0, 1.0, 0.0};
    const Vec3 start = center - axle * (width * 0.5);
    const Vec3 end = center + axle * (width * 0.5);

    DrawCylinderEx(toRaylib(start), toRaylib(end),
                   static_cast<float>(radius),
                   static_cast<float>(radius),
                   24,
                   Color{35, 38, 42, 255});
    DrawCylinderWiresEx(toRaylib(start), toRaylib(end),
                        static_cast<float>(radius),
                        static_cast<float>(radius),
                        24,
                        Color{210, 215, 220, 255});
}

void drawVehicle(const Vehicle& vehicle)
{
    const Vec3 bodyPosition = vehicle.State.GlobalVehicleMotion.frame_velocity.frame_position.translation;
    const Vec3 bodyAttitude = vehicle.State.GlobalVehicleMotion.frame_velocity.frame_position.orientation;

    Vec3 chassisSize{4.0, 2.0, 1.2};
    Vec3 chassisCenter{0.0, 0.0, 0.6};
    if (vehicle.Spec.Chassis != nullptr)
    {
        chassisSize = Vec3{vehicle.Spec.Chassis->L, vehicle.Spec.Chassis->W, vehicle.Spec.Chassis->H};
        chassisCenter = vehicle.Spec.Chassis->Center.translation;
    }

    drawOrientedBox(bodyPosition, bodyAttitude, chassisCenter, chassisSize, Color{35, 85, 155, 255});
    drawBasis(bodyPosition, bodyAttitude, 1.2);

    const double wheelRadius = vehicle.Spec.Wheel != nullptr ? vehicle.Spec.Wheel->R : 0.35;
    const double wheelWidth = vehicle.Spec.Wheel != nullptr ? vehicle.Spec.Wheel->H : 0.2;

    for (int wheel = 0; wheel < vehicle.Spec.TotalWheels; ++wheel)
    {
        const acceleration& wheelMotion = vehicle.State.GlobalWheelMountMotion[wheel];
        const acceleration& suspensionMotion = vehicle.State.GlobalSuspensionMotion[wheel];

        drawWheel(wheelMotion, wheelRadius, wheelWidth);
        DrawSphere(toRaylib(suspensionMotion.frame_velocity.frame_position.translation), 0.045f, ORANGE);
        DrawLine3D(toRaylib(suspensionMotion.frame_velocity.frame_position.translation),
                   toRaylib(wheelMotion.frame_velocity.frame_position.translation),
                   Color{220, 145, 40, 255});
    }
}
}

int main(int argc, char** argv)
{
    bool headless = false;
    std::string configPath = defaultVehicleConfig();
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--headless")
            headless = true;
        else
            configPath = arg;
    }

    Vehicle vehicle(configPath);

    if (vehicle.Spec.TotalWheels <= 0)
    {
        std::cerr << "VehicleDynamics: no wheels loaded from " << configPath << std::endl;
        return 1;
    }

    if (headless)
    {
        for (int step = 0; step < 2000; ++step)
            vehicle.Update();

        const Vec3 p = vehicle.State.GlobalVehicleMotion.frame_velocity.frame_position.translation;
        const Vec3 v = vehicle.State.GlobalVehicleMotion.frame_velocity.translation;
        const Vec3 rpy = vehicle.State.GlobalVehicleMotion.frame_velocity.frame_position.orientation;
        std::cout << "vehicle pos: " << p.x << ", " << p.y << ", " << p.z << std::endl;
        std::cout << "vehicle vel: " << v.x << ", " << v.y << ", " << v.z << std::endl;
        std::cout << "vehicle rpy deg: " << radToDeg(rpy.x) << ", "
                  << radToDeg(rpy.y) << ", " << radToDeg(rpy.z) << std::endl;
        return 0;
    }

    InitWindow(1280, 720, "VehicleDynamics Raylib");
    SetTargetFPS(60);

    Camera3D camera{};
    CameraControl cameraControl;
    bool paused = false;

    while (!WindowShouldClose())
    {
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;

        handleCameraInput(cameraControl);
        if (!paused)
        {
            for (int substep = 0; substep < 8; ++substep)
                vehicle.Update();
        }

        const Vec3 bodyPosition = vehicle.State.GlobalVehicleMotion.frame_velocity.frame_position.translation;
        updateCamera(camera, cameraControl, bodyPosition + Vec3{0.0, 0.0, 0.8});

        BeginDrawing();
        ClearBackground(Color{235, 238, 241, 255});

        BeginMode3D(camera);
        DrawGrid(40, 1.0f);
        DrawPlane(Vector3{0.0f, 0.0f, 0.0f}, Vector2{60.0f, 60.0f}, Color{205, 215, 205, 255});
        drawVehicle(vehicle);
        EndMode3D();

        const Vec3 velocity = vehicle.State.GlobalVehicleMotion.frame_velocity.translation;
        const Vec3 attitude = vehicle.State.GlobalVehicleMotion.frame_velocity.frame_position.orientation;
        DrawText(TextFormat("config %s", configPath.c_str()), 20, 20, 18, DARKGRAY);
        DrawText(TextFormat("pos %.2f %.2f %.2f", bodyPosition.x, bodyPosition.y, bodyPosition.z), 20, 46, 20, DARKGRAY);
        DrawText(TextFormat("vel %.2f %.2f %.2f", velocity.x, velocity.y, velocity.z), 20, 72, 20, DARKGRAY);
        DrawText(TextFormat("roll %.1f pitch %.1f yaw %.1f",
                            radToDeg(attitude.x), radToDeg(attitude.y), radToDeg(attitude.z)),
                 20, 98, 20, DARKGRAY);
        DrawText(TextFormat("wheels %d", vehicle.Spec.TotalWheels), 20, 124, 20, DARKGRAY);
        DrawText(paused ? "paused" : "running", 20, 150, 20, paused ? MAROON : GREEN);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
