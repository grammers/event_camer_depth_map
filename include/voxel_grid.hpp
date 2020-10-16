#pragma once 
#include <vector> 
#include <cmath>

namespace GRID{

class Voxel{
public:
    Voxel(int dimX, int dimY, int dimZ);

    void add_ray(double *cam_pos, double *event_dir);

private:

    int dimX;
    int dimY;
    int dimZ;

    double init_pos [6];

    // accses (x, y, z) grid[x + dimX * (y + dimY * z)]
    std::vector<int> grid;

};
} //namespace
