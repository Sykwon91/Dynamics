#include "TerrainMechanics.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace
{
constexpr double kSmall = 1e-9;

double clampDouble(double value, double lo, double hi)
{
    return std::max(lo, std::min(value, hi));
}
}

long long TerrainMechanics::zoneIndex(double value)
{
    return static_cast<long long>(std::floor(value / zone_size));
}

Vec3 TerrainMechanics::computeNormal(const Vec3& v0, const Vec3& v1, const Vec3& v2)
{
    const Vec3 e1 = v1 - v0;
    const Vec3 e2 = v2 - v0;
    return e1.cross(e2).normalized(1e-12);
}

bool TerrainMechanics::barycentricHeight(const Vec3& p0,
                                         const Vec3& p1,
                                         const Vec3& p2,
                                         double x,
                                         double z,
                                         double& height)
{
    const double den = (p1.z - p2.z) * (p0.x - p2.x)
                     + (p2.x - p1.x) * (p0.z - p2.z);
    if (std::fabs(den) < kSmall) return false;

    const double a = ((p1.z - p2.z) * (x - p2.x)
                    + (p2.x - p1.x) * (z - p2.z)) / den;
    const double b = ((p2.z - p0.z) * (x - p2.x)
                    + (p0.x - p2.x) * (z - p2.z)) / den;
    const double c = 1.0 - a - b;

    constexpr double eps = 1e-7;
    if (a < -eps || b < -eps || c < -eps) return false;

    height = a * p0.y + b * p1.y + c * p2.y;
    return true;
}

void TerrainMechanics::rebuildTrianglesFromVertices()
{
    for (Triangle& tri : triangles)
    {
        tri.normal = computeNormal(vertices[tri.i0], vertices[tri.i1], vertices[tri.i2]);
    }
}

void TerrainMechanics::buildZones()
{
    zones.clear();
    vertexZones.clear();

    for (int index = 0; index < static_cast<int>(vertices.size()); ++index)
    {
        vertexZones[ZoneCoord{zoneIndex(vertices[index].x), zoneIndex(vertices[index].z)}].push_back(index);
    }

    for (int index = 0; index < static_cast<int>(triangles.size()); ++index)
    {
        const Triangle& tri = triangles[index];
        const Vec3& v0 = vertices[tri.i0];
        const Vec3& v1 = vertices[tri.i1];
        const Vec3& v2 = vertices[tri.i2];
        const double minX = std::min({v0.x, v1.x, v2.x});
        const double maxX = std::max({v0.x, v1.x, v2.x});
        const double minZ = std::min({v0.z, v1.z, v2.z});
        const double maxZ = std::max({v0.z, v1.z, v2.z});

        for (long long zx = zoneIndex(minX); zx <= zoneIndex(maxX); ++zx)
        {
            for (long long zz = zoneIndex(minZ); zz <= zoneIndex(maxZ); ++zz)
            {
                zones[ZoneCoord{zx, zz}].push_back(index);
            }
        }
    }
}

void TerrainMechanics::clear()
{
    baseVertices.clear();
    vertices.clear();
    sinkage.clear();
    triangles.clear();
    zones.clear();
    vertexZones.clear();
    num_vertices = 0;
    num_faces = 0;
}

int TerrainMechanics::loadPLY(const std::string& filename)
{
    std::ifstream map(filename);
    std::string line;
    clear();

    if (!map.is_open())
    {
        std::cerr << "Failed to open PLY file: " << filename << std::endl;
        return 0;
    }

    while (std::getline(map, line))
    {
        if (line.find("format ascii") != std::string::npos)
            std::cout << line << std::endl;
        if (line.find("element vertex") != std::string::npos)
        {
            sscanf(line.c_str(), "element vertex %d", &num_vertices);
            std::cout << "element vertex : " << num_vertices << std::endl;
        }
        if (line.find("element face") != std::string::npos)
        {
            sscanf(line.c_str(), "element face %d", &num_faces);
            std::cout << "element face : " << num_faces << std::endl;
        }
        if (line == "end_header")
            break;
    }

    baseVertices.reserve(num_vertices);
    vertices.reserve(num_vertices);

    for (int i = 0; i < num_vertices; ++i)
    {
        Vec3 v;
        if (!std::getline(map, line))
        {
            std::cerr << "Unexpected end of file while reading vertex " << i << std::endl;
            return 0;
        }

        std::istringstream vertex_stream(line);
        if (!(vertex_stream >> v.x >> v.y >> v.z))
        {
            std::cerr << "Failed to parse vertex " << i << std::endl;
            return 0;
        }

        baseVertices.push_back(v);
        vertices.push_back(v);
    }

    sinkage.assign(vertices.size(), 0.0);

    for (int faceIndex = 0; faceIndex < num_faces; ++faceIndex)
    {
        if (!std::getline(map, line))
        {
            std::cerr << "Unexpected end of file while reading face " << faceIndex << std::endl;
            return 0;
        }

        std::istringstream face_stream(line);
        int n = 0;
        if (!(face_stream >> n) || n < 3)
        {
            std::cerr << "Failed to parse face " << faceIndex << std::endl;
            return 0;
        }

        std::vector<int> indices(static_cast<std::size_t>(n));
        for (int& index : indices)
        {
            if (!(face_stream >> index))
            {
                std::cerr << "Failed to parse face vertex index at face " << faceIndex << std::endl;
                return 0;
            }
            if (index < 0 || index >= static_cast<int>(vertices.size()))
            {
                std::cerr << "Face " << faceIndex << " has out-of-range vertex index " << index << std::endl;
                return 0;
            }
        }

        for (int i = 1; i + 1 < n; ++i)
        {
            Triangle tri;
            tri.i0 = indices[0];
            tri.i1 = indices[i];
            tri.i2 = indices[i + 1];
            triangles.push_back(tri);
        }
    }

    rebuildTrianglesFromVertices();
    buildZones();
    return 1;
}

int TerrainMechanics::loadPLYPatch(const std::string& filename, const Vec3& center, double radius)
{
    std::ifstream map(filename);
    std::string line;
    clear();

    if (!map.is_open())
    {
        std::cerr << "Failed to open PLY file: " << filename << std::endl;
        return 0;
    }

    while (std::getline(map, line))
    {
        if (line.find("format ascii") != std::string::npos)
            std::cout << line << std::endl;
        if (line.find("element vertex") != std::string::npos)
        {
            sscanf(line.c_str(), "element vertex %d", &num_vertices);
            std::cout << "element vertex : " << num_vertices << std::endl;
        }
        if (line.find("element face") != std::string::npos)
        {
            sscanf(line.c_str(), "element face %d", &num_faces);
            std::cout << "element face : " << num_faces << std::endl;
        }
        if (line == "end_header")
            break;
    }

    const double radius2 = radius * radius;
    std::unordered_map<int, int> remap;
    remap.reserve(200000);

    for (int i = 0; i < num_vertices; ++i)
    {
        Vec3 v;
        if (!std::getline(map, line)) return 0;
        std::istringstream vertex_stream(line);
        if (!(vertex_stream >> v.x >> v.y >> v.z)) return 0;

        const double dx = v.x - center.x;
        const double dz = v.z - center.z;
        if (dx * dx + dz * dz <= radius2)
        {
            const int newIndex = static_cast<int>(vertices.size());
            remap[i] = newIndex;
            baseVertices.push_back(v);
            vertices.push_back(v);
        }
    }

    sinkage.assign(vertices.size(), 0.0);

    for (int faceIndex = 0; faceIndex < num_faces; ++faceIndex)
    {
        if (!std::getline(map, line)) return 0;
        std::istringstream face_stream(line);
        int n = 0;
        if (!(face_stream >> n) || n < 3) continue;

        std::vector<int> original(static_cast<std::size_t>(n));
        bool keep = true;
        for (int& index : original)
        {
            face_stream >> index;
            if (remap.find(index) == remap.end())
                keep = false;
        }
        if (!keep) continue;

        for (int i = 1; i + 1 < n; ++i)
        {
            Triangle tri;
            tri.i0 = remap[original[0]];
            tri.i1 = remap[original[i]];
            tri.i2 = remap[original[i + 1]];
            triangles.push_back(tri);
        }
    }

    num_vertices = static_cast<int>(vertices.size());
    num_faces = static_cast<int>(triangles.size());
    rebuildTrianglesFromVertices();
    buildZones();
    std::cout << "patch vertex : " << num_vertices << std::endl;
    std::cout << "patch triangle : " << num_faces << std::endl;
    return !vertices.empty() && !triangles.empty();
}

bool TerrainMechanics::queryHeight(double x, double z, double& height, Vec3* normal) const
{
    const auto zone_iter = zones.find(ZoneCoord{zoneIndex(x), zoneIndex(z)});
    if (zone_iter == zones.end()) return false;

    bool found = false;
    double bestHeight = -std::numeric_limits<double>::infinity();
    Vec3 bestNormal{0.0, 1.0, 0.0};

    for (int triangleIndex : zone_iter->second)
    {
        const Triangle& tri = triangles[triangleIndex];
        const Vec3& v0 = vertices[tri.i0];
        const Vec3& v1 = vertices[tri.i1];
        const Vec3& v2 = vertices[tri.i2];
        double candidateHeight = 0.0;
        if (!barycentricHeight(v0, v1, v2, x, z, candidateHeight)) continue;

        if (!found || candidateHeight > bestHeight)
        {
            found = true;
            bestHeight = candidateHeight;
            bestNormal = tri.normal;
        }
    }

    if (!found) return false;

    height = bestHeight;
    if (normal) *normal = bestNormal;
    return true;
}

TerrainContactResult TerrainMechanics::applySMCContact(const Vec3& contactPoint,
                                                       double contactRadius,
                                                       double normalLoad,
                                                       double sinkVelocity,
                                                       double dt)
{
    TerrainContactResult result;
    if (contactRadius <= kSmall || dt <= 0.0) return result;

    Vec3 normal;
    if (!queryHeight(contactPoint.x, contactPoint.z, result.GroundHeight, &normal))
        return result;

    result.Contact = contactPoint.y <= result.GroundHeight;
    result.Sinkage = std::max(0.0, result.GroundHeight - contactPoint.y);

    const double width = std::max(2.0 * contactRadius, 0.01);
    const double bekkerPressure =
        (smc.BekkerKc / width + smc.BekkerKphi)
        * std::pow(std::max(result.Sinkage, 0.0), smc.BekkerExponent);
    const double dampingPressure = smc.Damping * std::max(0.0, sinkVelocity);
    const double loadPressure = normalLoad / (3.14159265358979323846 * contactRadius * contactRadius);

    result.Pressure = std::max(loadPressure, bekkerPressure + dampingPressure);
    result.NormalForce = result.Pressure * 3.14159265358979323846 * contactRadius * contactRadius;

    const double deformationIncrement = result.Pressure * smc.PlasticGain * dt;
    bool changed = false;

    const long long minZoneX = zoneIndex(contactPoint.x - contactRadius);
    const long long maxZoneX = zoneIndex(contactPoint.x + contactRadius);
    const long long minZoneZ = zoneIndex(contactPoint.z - contactRadius);
    const long long maxZoneZ = zoneIndex(contactPoint.z + contactRadius);

    for (long long zx = minZoneX; zx <= maxZoneX; ++zx)
    {
        for (long long zz = minZoneZ; zz <= maxZoneZ; ++zz)
        {
            const auto iter = vertexZones.find(ZoneCoord{zx, zz});
            if (iter == vertexZones.end()) continue;

            for (int vertexIndex : iter->second)
            {
                const std::size_t i = static_cast<std::size_t>(vertexIndex);
                const double dx = baseVertices[i].x - contactPoint.x;
                const double dz = baseVertices[i].z - contactPoint.z;
                const double distance = std::sqrt(dx * dx + dz * dz);
                if (distance > contactRadius) continue;

                const double falloff = 0.5 * (1.0 + std::cos(3.14159265358979323846 * distance / contactRadius));
                const double targetSinkage = clampDouble(sinkage[i] + deformationIncrement * falloff,
                                                         0.0,
                                                         smc.MaxSinkage);
                if (targetSinkage > sinkage[i])
                {
                    sinkage[i] = targetSinkage;
                    vertices[i].y = baseVertices[i].y - sinkage[i];
                    changed = true;
                }
            }
        }
    }

    if (changed)
    {
        rebuildTrianglesFromVertices();
        buildZones();
    }

    return result;
}

int TerrainMechanics::vertexCount() const
{
    return static_cast<int>(vertices.size());
}

int TerrainMechanics::faceCount() const
{
    return static_cast<int>(triangles.size());
}

const std::vector<Vec3>& TerrainMechanics::getVertices() const
{
    return vertices;
}

const std::vector<Vec3>& TerrainMechanics::getBaseVertices() const
{
    return baseVertices;
}

const std::vector<double>& TerrainMechanics::getSinkage() const
{
    return sinkage;
}

const std::vector<TerrainMechanics::Triangle>& TerrainMechanics::getTriangles() const
{
    return triangles;
}

bool TerrainMechanics::intersectRayTriangle(const Vec3& orig, const Vec3& dir, const Triangle& tri, float& t)
{
    const Vec3& v0 = vertices[tri.i0];
    const Vec3& v1 = vertices[tri.i1];
    const Vec3& v2 = vertices[tri.i2];
    const Vec3 edge1 = v1 - v0;
    const Vec3 edge2 = v2 - v0;
    const Vec3 h = dir.cross(edge2);
    const double a = edge1.dot(h);
    if (std::fabs(a) < kSmall) return false;

    const double f = 1.0 / a;
    const Vec3 s = orig - v0;
    const double u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return false;

    const Vec3 q = s.cross(edge1);
    const double v = f * dir.dot(q);
    if (v < 0.0 || u + v > 1.0) return false;

    const double rayT = f * edge2.dot(q);
    if (rayT <= kSmall) return false;

    t = static_cast<float>(rayT);
    return true;
}

bool TerrainMechanics::getHeightAndNormal(Vec3& objectOrigin)
{
    Vec3 normal;
    const bool ok = queryHeight(objectOrigin.x, objectOrigin.z, objectOrigin.y, &normal);
    if (ok)
    {
        std::cout << objectOrigin.x << ", "
                  << objectOrigin.y << ", "
                  << objectOrigin.z << std::endl;
    }
    return ok;
}
