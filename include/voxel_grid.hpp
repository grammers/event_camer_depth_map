#pragma once 
#include <vector> 
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <median_filter.hpp>
#include <position.hpp>

//for debug
#include "ros/ros.h"
#define RESULUTION 33
#define MAX_DEPTH 176

namespace GRID{

class Voxel{
public:
    Voxel(ODOM::Position *p, int dimX, int dimY, int dimZ);

    void add_ray(double *event_dir);
    int nr_ray(int x, int y, int z);
    void clear();
    void filter(int width, int height, double fx, double fy);

    int filtered_mark(int x, int y);
    void depth_map(cv::Mat& img);
    
private:

    ODOM::Position* pos;
    int dim[3];
    double init_pos [3];
    // accses (x, y, z) grid[x + dimX * (y + dimY * z)]
    std::vector<int> grid;

    void setup(double *camera_pos, double *event_dir_vector, double *ray);
    int ray_direction(double p);
    bool add_hit(double t, double *ray);
    void hit_id(double t, double *ray, int *index);
    bool in_bound(int *index);
    double distans(int *index, double *ray);
    void max_nr_ray(double *direction, int w, int h);
    
    cv::Mat max_dist;

    // calibration
    float med;
    int nr;



};
} //namespace
