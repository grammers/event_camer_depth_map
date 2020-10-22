#pragma once 
#include <vector> 
#include <cmath>

//for debug
#include "ros/ros.h"
#define resulution 50

namespace GRID{

class Voxel{
public:
    Voxel(int dimX, int dimY, int dimZ);

    void add_ray(double *cam_pos, double *event_dir);
    double depth_at_pixel(double *cam_pos, double *pixel_vector);

private:

    int dimX;
    int dimY;
    int dimZ;
    int dim[3];
    double init_pos [6];

    // accses (x, y, z) grid[x + dimX * (y + dimY * z)]
    std::vector<int> grid;
    
    int ray_direction(double p);
    bool add_hit(double t, double *ray);
    double ray_hit(int plain, double pos, double delta);
    bool in_bound(int *index);

    double distans(int *index, double *ray);
    void hit_id(double t, double *ray, int *index);
    void setup(double *camera_pos, double *event_dir_vector, double *ray);

};
} //namespace
