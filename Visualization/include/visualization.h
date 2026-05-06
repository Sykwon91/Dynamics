#pragma once

#include "collision.h"
#include <iosfwd>
#include <string>

namespace Visualization
{
    class ObjWriter
    {
        public:
            explicit ObjWriter(std::ostream& output);

            void writeBox(const std::string& name, const Box& box);
            void writeCylinder(const std::string& name, const Cylinder& cylinder, int segments = 24);

        private:
            std::ostream& output;
            int nextVertexIndex{1};
    };

    bool writeBoxObjFile(const std::string& filename, const std::string& name, const Box& box);
    bool writeCylinderObjFile(const std::string& filename, const std::string& name, const Cylinder& cylinder, int segments = 24);
}
