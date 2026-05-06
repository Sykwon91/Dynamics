#include "visualization.h"

#include "linearalgebra.h"
#include <cmath>
#include <fstream>
#include <ostream>
#include <vector>

namespace
{
    constexpr double Pi = 3.14159265358979323846;

    Mat3 rotationFromEuler(const Vec3& rotation)
    {
        Mat3 rx, ry, rz;
        rx.Rx(rotation.x);
        ry.Ry(rotation.y);
        rz.Rz(rotation.z);
        return rz * ry * rx;
    }

    Vec3 transformPoint(const Position& pose, const Vec3& local)
    {
        return pose.translation + rotationFromEuler(pose.rotation) * local;
    }

    void writeVertex(std::ostream& output, const Vec3& point)
    {
        output << "v " << point.x << ' ' << point.y << ' ' << point.z << '\n';
    }
}

namespace Visualization
{
    ObjWriter::ObjWriter(std::ostream& output)
        : output(output)
    {
    }

    void ObjWriter::writeBox(const std::string& name, const Box& box)
    {
        output << "o " << name << '\n';

        const Vec3 localVertices[8] = {
            { box.L * 0.5,  box.W * 0.5,  box.H * 0.5 },
            { box.L * 0.5, -box.W * 0.5,  box.H * 0.5 },
            {-box.L * 0.5, -box.W * 0.5,  box.H * 0.5 },
            {-box.L * 0.5,  box.W * 0.5,  box.H * 0.5 },
            { box.L * 0.5,  box.W * 0.5, -box.H * 0.5 },
            { box.L * 0.5, -box.W * 0.5, -box.H * 0.5 },
            {-box.L * 0.5, -box.W * 0.5, -box.H * 0.5 },
            {-box.L * 0.5,  box.W * 0.5, -box.H * 0.5 }
        };

        for (const Vec3& vertex : localVertices)
            writeVertex(output, transformPoint(box.Center, vertex));

        const int base = nextVertexIndex;
        output << "f " << base + 0 << ' ' << base + 1 << ' ' << base + 2 << ' ' << base + 3 << '\n';
        output << "f " << base + 4 << ' ' << base + 7 << ' ' << base + 6 << ' ' << base + 5 << '\n';
        output << "f " << base + 0 << ' ' << base + 4 << ' ' << base + 5 << ' ' << base + 1 << '\n';
        output << "f " << base + 1 << ' ' << base + 5 << ' ' << base + 6 << ' ' << base + 2 << '\n';
        output << "f " << base + 2 << ' ' << base + 6 << ' ' << base + 7 << ' ' << base + 3 << '\n';
        output << "f " << base + 3 << ' ' << base + 7 << ' ' << base + 4 << ' ' << base + 0 << '\n';

        nextVertexIndex += 8;
    }

    void ObjWriter::writeCylinder(const std::string& name, const Cylinder& cylinder, int segments)
    {
        if (segments < 3)
            segments = 3;

        output << "o " << name << '\n';

        const double halfHeight = cylinder.H * 0.5;
        std::vector<Vec3> bottom;
        std::vector<Vec3> top;
        bottom.reserve(static_cast<std::size_t>(segments));
        top.reserve(static_cast<std::size_t>(segments));

        for (int i = 0; i < segments; ++i)
        {
            const double angle = 2.0 * Pi * static_cast<double>(i) / static_cast<double>(segments);
            const double c = std::cos(angle);
            const double s = std::sin(angle);
            bottom.push_back(transformPoint(cylinder.Center, { cylinder.R * c, -halfHeight, cylinder.R * s }));
            top.push_back(transformPoint(cylinder.Center, { cylinder.R * c, halfHeight, cylinder.R * s }));
        }

        for (const Vec3& vertex : bottom)
            writeVertex(output, vertex);
        for (const Vec3& vertex : top)
            writeVertex(output, vertex);

        const int base = nextVertexIndex;
        output << "f";
        for (int i = segments - 1; i >= 0; --i)
            output << ' ' << base + i;
        output << '\n';

        output << "f";
        for (int i = 0; i < segments; ++i)
            output << ' ' << base + segments + i;
        output << '\n';

        for (int i = 0; i < segments; ++i)
        {
            const int next = (i + 1) % segments;
            output << "f " << base + i << ' ' << base + next << ' '
                   << base + segments + next << ' ' << base + segments + i << '\n';
        }

        nextVertexIndex += segments * 2;
    }

    bool writeBoxObjFile(const std::string& filename, const std::string& name, const Box& box)
    {
        std::ofstream file(filename);
        if (!file)
            return false;

        ObjWriter writer(file);
        writer.writeBox(name, box);
        return true;
    }

    bool writeCylinderObjFile(const std::string& filename, const std::string& name, const Cylinder& cylinder, int segments)
    {
        std::ofstream file(filename);
        if (!file)
            return false;

        ObjWriter writer(file);
        writer.writeCylinder(name, cylinder, segments);
        return true;
    }
}
