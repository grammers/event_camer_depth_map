#pragma once 
#include <vector> 
#include <cmath>
#include <opencv2/imgproc.hpp>

//for debug
#include "ros/ros.h"
#define resulution 33

namespace GRID{

class Voxel{
public:
    Voxel(int dimX, int dimY, int dimZ);

    void add_ray(double *cam_pos, double *event_dir);
    double depth_at_pixel(double *cam_pos, double *pixel_vector);

    bool is_marked(int x, int y, int z);
    void normalise();
    int nr_ray(int x, int y, int z);

    void filter();
    int filtered_mark(int x, int y);
private:

    int dimX;
    int dimY;
    int dimZ;
    int dim[3];
    double init_pos [6];

    // accses (x, y, z) grid[x + dimX * (y + dimY * z)]
    std::vector<int> grid;
    //std::vector<bool> marker;
    int max_ray;
    
    int ray_direction(double p);
    bool add_hit(double t, double *ray);
    double ray_hit(int plain, double pos, double delta);
    bool in_bound(int *index);

    //filering stuf
    cv::Mat max;
    cv::Mat max_coordinates;
    cv::Mat max_filtered_coordinates;
    cv::Mat mask;


    double distans(int *index, double *ray);
    void hit_id(double t, double *ray, int *index);
    void setup(double *camera_pos, double *event_dir_vector, double *ray);

};
} //namespace
