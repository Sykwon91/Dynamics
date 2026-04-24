#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include "linearalgebra.h"




class Terrain
{
    private:
        struct HeightSample
        {
            Vec3 position;
        };

        struct ZoneCoord
        {
            long long x;
            long long y;

            bool operator==(const ZoneCoord& other) const
            {
                return x == other.x && y == other.y;
            }
        };

        struct ZoneCoordHash
        {
            std::size_t operator()(const ZoneCoord& coord) const
            {
                const auto hx = std::hash<long long>{}(coord.x);
                const auto hy = std::hash<long long>{}(coord.y);
                return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6) + (hx >> 2));
            }
        };

        struct Triangle
        {
            Vec3 v0;
            Vec3 v1;
            Vec3 v2;
            Vec3 normal;
        };

        int num_vertices{};
        int num_faces{};
        std::vector<Vec3> vertices;
        std::vector<Triangle> triangles;
        std::unordered_map<ZoneCoord, std::vector<HeightSample>, ZoneCoordHash> zones;

        static constexpr double zone_size = 1.0;

        static long long zoneIndex(double value);
        void buildZones();
    public:
        int loadPLY(const std::string& filename);
        int vertexCount() const;
        int faceCount() const;
        const std::vector<Vec3>& getVertices() const;
        const std::vector<Triangle>& getTriangles() const;
        bool intersectRayTriangle(const Vec3& orig,const Vec3& dir,const Triangle& tri,float& t); 
        bool getHeightAndNormal(Vec3& ObjectOrigin) ;
    
};

using TerrainMechanics = Terrain;
