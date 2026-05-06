#include "TerrainMechanics.h"
#include <chrono>

int main()
{
    Terrain terrain1;
    terrain1.loadPLY("../Maps/1GY_roadquary.ply");
    Vec3 test = {-2955.21,0,-1884.32};
    std::chrono::high_resolution_clock timer11;
    std::chrono::time_point<std::chrono::high_resolution_clock> now =  timer11.now();
    terrain1.getHeightAndNormal(test);
    std::chrono::time_point<std::chrono::high_resolution_clock> end =  timer11.now();
    std::cout << (end - now).count() << std::endl;
}
