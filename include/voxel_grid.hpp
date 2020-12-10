#pragma once 
#include <vector> 
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <median_filter.hpp>
#include <event_obj.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cnpy.h>

#include <image_geometry/pinhole_camera_model.h>
#include <geometry_utils.hpp>
#include <trajectory.hpp>

//for debug
#include "ros/ros.h"

#define DEPTH 0.02
#define Z0 0.5
#define MED_FILTER 5
#define KERNAL 5
#define ADAPT 10.0

namespace GRID{

class Voxel{
public:
    Voxel(int dimX, int dimY, int dimZ);

    void add_ray_traj(EVENTOBJ::EventObj *e, LinearTrajectory *trajectory, geometry_utils::Transformation *T_cw);
    void clear();
    void filter();
    void camInit(const sensor_msgs::CameraInfo *msg);

    void depth_map(cv::Mat& img);
    void npy();
    
private:

    int dim[3];
    int FX;
    int FY;
    int CX;
    int CY;
    Eigen::Matrix3f K;


    // accses (x, y, z) grid[x + dimX * (y + dimY * z)]
    std::vector<int> grid;

    bool in_bound(int *index);
    int nr_ray(int x, int y, int z);
    
    cv::Mat max_dist;

    image_geometry::PinholeCameraModel dvs_cam;

};
} //namespace
