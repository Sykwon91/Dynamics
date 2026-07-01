#include "TerrainMechanics.h"

#include "raylib.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace
{
Vector3 toRaylib(const Vec3& value)
{
    return Vector3{
        static_cast<float>(value.x),
        static_cast<float>(value.y),
        static_cast<float>(value.z)
    };
}

Color sinkageColor(double value)
{
    const double t = std::clamp(value / 0.45, 0.0, 1.0);
    return Color{
        static_cast<unsigned char>(80 + 130 * t),
        static_cast<unsigned char>(145 - 70 * t),
        static_cast<unsigned char>(70 - 35 * t),
        255
    };
}

struct TerrainRenderMesh
{
    Mesh mesh{};
    Model model{};
    std::vector<float> vertices;
    std::vector<float> normals;
    bool loaded = false;
};

void fillRenderBuffers(const Terrain& terrain, TerrainRenderMesh& render)
{
    const auto& triangles = terrain.getTriangles();
    const auto& terrainVertices = terrain.getVertices();
    render.vertices.resize(triangles.size() * 9);
    render.normals.resize(triangles.size() * 9);

    for (std::size_t i = 0; i < triangles.size(); ++i)
    {
        const Terrain::Triangle& tri = triangles[i];
        const Vec3 points[3] = {
            terrainVertices[static_cast<std::size_t>(tri.i0)],
            terrainVertices[static_cast<std::size_t>(tri.i1)],
            terrainVertices[static_cast<std::size_t>(tri.i2)]
        };

        for (int corner = 0; corner < 3; ++corner)
        {
            const std::size_t base = i * 9 + corner * 3;
            render.vertices[base + 0] = static_cast<float>(points[corner].x);
            render.vertices[base + 1] = static_cast<float>(points[corner].y);
            render.vertices[base + 2] = static_cast<float>(points[corner].z);
            render.normals[base + 0] = static_cast<float>(tri.normal.x);
            render.normals[base + 1] = static_cast<float>(tri.normal.y);
            render.normals[base + 2] = static_cast<float>(tri.normal.z);
        }
    }
}

TerrainRenderMesh createRenderMesh(const Terrain& terrain)
{
    TerrainRenderMesh render;
    fillRenderBuffers(terrain, render);

    render.mesh.vertexCount = static_cast<int>(render.vertices.size() / 3);
    render.mesh.triangleCount = static_cast<int>(render.vertices.size() / 9);
    render.mesh.vertices = render.vertices.data();
    render.mesh.normals = render.normals.data();
    render.mesh.indices = nullptr;
    UploadMesh(&render.mesh, false);
    render.model = LoadModelFromMesh(render.mesh);
    render.loaded = true;
    return render;
}

void updateRenderMesh(const Terrain& terrain, TerrainRenderMesh& render)
{
    fillRenderBuffers(terrain, render);
    UpdateMeshBuffer(render.mesh, 0, render.vertices.data(), static_cast<int>(render.vertices.size() * sizeof(float)), 0);
    UpdateMeshBuffer(render.mesh, 2, render.normals.data(), static_cast<int>(render.normals.size() * sizeof(float)), 0);
}
}

int main()
{
    Terrain terrain;
    Vec3 contactPoint{-2955.21, 0.0, -1884.32};
    if (!terrain.loadPLYPatch("Maps/1GY_roadquary.ply", contactPoint, 35.0)
        && !terrain.loadPLYPatch("../Maps/1GY_roadquary.ply", contactPoint, 35.0))
    {
        return 1;
    }

    terrain.queryHeight(contactPoint.x, contactPoint.z, contactPoint.y);
    contactPoint.y -= 0.08;

    InitWindow(1280, 720, "Terrain SMC Mechanics");
    SetTargetFPS(60);

    Camera3D camera{};
    camera.position = Vector3{static_cast<float>(contactPoint.x + 18.0), static_cast<float>(contactPoint.y + 12.0), static_cast<float>(contactPoint.z + 18.0)};
    camera.target = toRaylib(contactPoint);
    camera.up = Vector3{0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    TerrainRenderMesh render = createRenderMesh(terrain);
    double wheelPhase = 0.0;
    TerrainContactResult contact;

    while (!WindowShouldClose())
    {
        const double dt = static_cast<double>(GetFrameTime());
        wheelPhase += dt;

        if (IsKeyDown(KEY_LEFT)) contactPoint.x -= 4.0 * dt;
        if (IsKeyDown(KEY_RIGHT)) contactPoint.x += 4.0 * dt;
        if (IsKeyDown(KEY_UP)) contactPoint.z -= 4.0 * dt;
        if (IsKeyDown(KEY_DOWN)) contactPoint.z += 4.0 * dt;

        double groundHeight = contactPoint.y;
        terrain.queryHeight(contactPoint.x, contactPoint.z, groundHeight);
        contactPoint.y = groundHeight - (0.08 + 0.04 * std::max(0.0, std::sin(wheelPhase * 2.0)));

        contact = terrain.applySMCContact(contactPoint, 1.2, 45000.0, 0.05, dt);
        updateRenderMesh(terrain, render);

        camera.target = toRaylib(contactPoint);
        if (IsKeyDown(KEY_A)) camera.position.x -= static_cast<float>(8.0 * dt);
        if (IsKeyDown(KEY_D)) camera.position.x += static_cast<float>(8.0 * dt);
        if (IsKeyDown(KEY_W)) camera.position.z -= static_cast<float>(8.0 * dt);
        if (IsKeyDown(KEY_S)) camera.position.z += static_cast<float>(8.0 * dt);

        BeginDrawing();
        ClearBackground(Color{235, 238, 241, 255});

        BeginMode3D(camera);
        DrawModel(render.model, Vector3{0.0f, 0.0f, 0.0f}, 1.0f, Color{112, 155, 91, 255});
        DrawModelWires(render.model, Vector3{0.0f, 0.0f, 0.0f}, 1.0f, Color{45, 65, 45, 80});
        DrawSphere(toRaylib(contactPoint), 0.45f, RED);
        DrawCylinder(toRaylib(Vec3{contactPoint.x, contactPoint.y + 0.65, contactPoint.z}),
                     0.65f,
                     0.65f,
                     0.28f,
                     24,
                     Color{80, 80, 85, 255});
        DrawGrid(20, 1.0f);
        EndMode3D();

        DrawText(TextFormat("vertices %d triangles %d", terrain.vertexCount(), terrain.faceCount()), 20, 20, 20, DARKGRAY);
        DrawText(TextFormat("ground %.3f sinkage %.3f", contact.GroundHeight, contact.Sinkage), 20, 46, 20, DARKGRAY);
        DrawText(TextFormat("pressure %.1f normal %.1f", contact.Pressure, contact.NormalForce), 20, 72, 20, DARKGRAY);
        DrawText("Arrow keys: move contact / WASD: move camera", 20, 98, 20, DARKGRAY);
        DrawText("SMC depression is permanent vertex sinkage", 20, 124, 20, MAROON);

        EndDrawing();
    }

    if (render.loaded)
        UnloadModel(render.model);
    CloseWindow();
    return 0;
}
