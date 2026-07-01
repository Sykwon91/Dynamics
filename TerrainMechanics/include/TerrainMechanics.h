#ifndef TERRAIN_MECHANICS_H
#define TERRAIN_MECHANICS_H

#include "linearalgebra.h"

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

struct TerrainSMCParameters
{
    double BekkerKc = 12000.0;
    double BekkerKphi = 180000.0;
    double BekkerExponent = 1.1;
    double Damping = 1200.0;
    double PlasticGain = 0.00008;
    double MaxSinkage = 0.45;
};

struct TerrainContactResult
{
    bool Contact = false;
    double GroundHeight = 0.0;
    double Sinkage = 0.0;
    double Pressure = 0.0;
    double NormalForce = 0.0;
};

class Terrain
{
public:
    struct Triangle
    {
        int i0 = 0;
        int i1 = 0;
        int i2 = 0;
        Vec3 normal;
    };

    int loadPLY(const std::string& filename);
    int loadPLYPatch(const std::string& filename, const Vec3& center, double radius);
    bool queryHeight(double x, double z, double& height, Vec3* normal = nullptr) const;
    TerrainContactResult applySMCContact(const Vec3& contactPoint,
                                         double contactRadius,
                                         double normalLoad,
                                         double sinkVelocity,
                                         double dt);

    int vertexCount() const;
    int faceCount() const;
    const std::vector<Vec3>& getVertices() const;
    const std::vector<Vec3>& getBaseVertices() const;
    const std::vector<double>& getSinkage() const;
    const std::vector<Triangle>& getTriangles() const;
    bool intersectRayTriangle(const Vec3& orig, const Vec3& dir, const Triangle& tri, float& t);
    bool getHeightAndNormal(Vec3& objectOrigin);

private:
    struct ZoneCoord
    {
        long long x = 0;
        long long z = 0;

        bool operator<(const ZoneCoord& other) const
        {
            return x < other.x || (x == other.x && z < other.z);
        }
    };

    int num_vertices = 0;
    int num_faces = 0;
    std::vector<Vec3> baseVertices;
    std::vector<Vec3> vertices;
    std::vector<double> sinkage;
    std::vector<Triangle> triangles;
    std::map<ZoneCoord, std::vector<int>> zones;
    std::map<ZoneCoord, std::vector<int>> vertexZones;
    TerrainSMCParameters smc;

    static constexpr double zone_size = 4.0;

    static long long zoneIndex(double value);
    static Vec3 computeNormal(const Vec3& v0, const Vec3& v1, const Vec3& v2);
    static bool barycentricHeight(const Vec3& p0,
                                  const Vec3& p1,
                                  const Vec3& p2,
                                  double x,
                                  double z,
                                  double& height);
    void rebuildTrianglesFromVertices();
    void buildZones();
    void clear();
};

using TerrainMechanics = Terrain;

#endif
