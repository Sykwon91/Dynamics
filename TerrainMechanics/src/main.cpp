#include "TerrainMechanics.h"
#include <chrono>

int main()
{
    Terrain terrain1;
    terrain1.loadPLY("/home/kwon/Downloads/add_point.ply");
    Vec3 test = {-26,300,1};
    std::chrono::high_resolution_clock timer11;
    std::chrono::time_point<std::chrono::high_resolution_clock> now =  timer11.now();
    terrain1.getHeightAndNormal(test);
    std::chrono::time_point<std::chrono::high_resolution_clock> end =  timer11.now();
    std::cout << (end - now).count() << std::endl;
}
