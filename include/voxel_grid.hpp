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
    
    int Voxel::ray_detection(double p);
    bool add_hit(double t, int start_x, int start_y, int start_z, double px, double py, double pz);
    double ray_hit(int plain, double pos, double delta);
    bool in_bound(int x, int y, int z);

};
} //namespace
