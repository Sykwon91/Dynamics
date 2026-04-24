#include "TerrainMechanics.h"
#include <algorithm>
#include <cmath>
#include <limits>

long long TerrainMechanics::zoneIndex(double value)
{
    return static_cast<long long>(std::floor(value / zone_size));
}

void TerrainMechanics::buildZones()
{
    zones.clear();
    zones.reserve(vertices.size());

    for (const auto& point : vertices)
    {
        const ZoneCoord coord{zoneIndex(point.x), zoneIndex(point.y)};
        zones[coord].push_back(HeightSample{point});
    }
}

int TerrainMechanics::loadPLY(const std::string& filename)
{
    std::ifstream map(filename);
    std::string line;
    vertices.clear();
    triangles.clear();
    zones.clear();

    while (std::getline(map,line))
    {
        if (line.find("format") != std::string::npos)
            {
                std::cout <<line.c_str() << std::endl;
            }
        if (line.find("element vertex") != std::string::npos)
            {
                sscanf(line.c_str(), "element vertex %d", &this->num_vertices);
                std::cout <<"element vertex : " << this->num_vertices << std::endl;
            }
        if (line.find("element face") != std::string::npos)
            {
                sscanf(line.c_str(), "element face %d", &this->num_faces);
                std::cout << "element face : " << this->num_faces << std::endl;
            }
        if (line == "end_header")
            break;
    }
    for (int i = 0; i < this->num_vertices; i++) 
    {
        Vec3 v;
        map >> v.x >> v.y >> v.z;
        this->vertices.push_back(v);
    }
    for (int i = 0; i < num_faces; i++) {
        int n, i0, i1, i2;
        map >> n >> i0 >> i1 >> i2;

        Triangle tri;
        tri.v0 = vertices[i0];
        tri.v1 = vertices[i1];
        tri.v2 = vertices[i2];

        // normal 계산
        Vec3 e1 = {tri.v1.x - tri.v0.x, tri.v1.y - tri.v0.y, tri.v1.z - tri.v0.z};
        Vec3 e2 = {tri.v2.x - tri.v0.x, tri.v2.y - tri.v0.y, tri.v2.z - tri.v0.z};

        tri.normal = 
        {
            e1.y * e2.z - e1.z * e2.y,
            e1.z * e2.x - e1.x * e2.z,
            e1.x * e2.y - e1.y * e2.x
        };

        triangles.push_back(tri);
    }

    buildZones();
    return 1;
}



bool TerrainMechanics::getHeightAndNormal(Vec3& ObjectOrigin) {

    const long long center_x = zoneIndex(ObjectOrigin.x);
    const long long center_y = zoneIndex(ObjectOrigin.y);
    const HeightSample* closest_sample = nullptr;

    struct Candidate
    {
        const HeightSample* sample;
        double distance;
    };

    std::vector<Candidate> candidates;
    candidates.reserve(32);

    for (long long dx = -2; dx <= 2; ++dx)
    {
        for (long long dy = -2; dy <= 2; ++dy)
        {
            const ZoneCoord coord{center_x + dx, center_y + dy};
            const auto zone_iter = zones.find(coord);
            if (zone_iter == zones.end())
            {
                continue;
            }
            for (const auto& sample : zone_iter->second)
            {
                const double diff_x = sample.position.x - ObjectOrigin.x;
                const double diff_y = sample.position.y - ObjectOrigin.y;
                const double distance = diff_x * diff_x + diff_y * diff_y;
                candidates.push_back(Candidate{&sample, distance});
            }
        }
    }

    if (candidates.empty())
    {
        return false;
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& left, const Candidate& right) {
        return left.distance < right.distance;
    });

    closest_sample = candidates.front().sample;

    if (candidates.size() >= 3)
    {
        const Vec3& p0 = candidates[0].sample->position;
        bool interpolated = false;

        for (std::size_t i = 1; i < candidates.size() && !interpolated; ++i)
        {
            for (std::size_t j = i + 1; j < candidates.size() && !interpolated; ++j)
            {
                const Vec3& p1 = candidates[i].sample->position;
                const Vec3& p2 = candidates[j].sample->position;

                const double det =
                    (p1.x - p0.x) * (p2.y - p0.y) -
                    (p2.x - p0.x) * (p1.y - p0.y);

                if (std::fabs(det) < 1e-9)
                {
                    continue;
                }

                const double a =
                    ((p1.z - p0.z) * (p2.y - p0.y) -
                     (p2.z - p0.z) * (p1.y - p0.y)) / det;
                const double b =
                    ((p1.x - p0.x) * (p2.z - p0.z) -
                     (p2.x - p0.x) * (p1.z - p0.z)) / det;
                const double c = p0.z - a * p0.x - b * p0.y;

                ObjectOrigin.z = a * ObjectOrigin.x + b * ObjectOrigin.y + c;
                interpolated = true;
            }
        }

        if (!interpolated)
        {
            ObjectOrigin.z = closest_sample->position.z;
        }
    }
    else
    {
        ObjectOrigin.z = closest_sample->position.z;
    }

    std::cout << ObjectOrigin.x << ", "
              << ObjectOrigin.y << ", "
              << ObjectOrigin.z << std::endl;
    return true;
}
