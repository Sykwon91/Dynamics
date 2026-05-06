#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <map>
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
            long long z;

            bool operator==(const ZoneCoord& other) const
            {
                return x == other.x && z == other.z;
            }
            bool operator<(const ZoneCoord& other) const
            {
                return x < other.x || (x == other.x && z < other.z);
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
        std::map<ZoneCoord, std::vector<HeightSample>> zones;

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
