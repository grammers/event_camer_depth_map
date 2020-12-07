#pragma once 
#include <vector> 
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <median_filter.hpp>
#include <position.hpp>
#include <event_obj.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cnpy.h>

#include <image_geometry/pinhole_camera_model.h>
#include <geometry_utils.hpp>
#include <trajectory.hpp>

//for debug
#include "ros/ros.h"
#define RESULUTION 33
#define MAX_DEPTH 176
#define DEPTH 0.025
#define Z0 0.5

namespace GRID{

class Voxel{
public:
    Voxel(ODOM::Position *p, int dimX, int dimY, int dimZ, int FX, int FY, int CX, int CY);

    void add_ray(EVENTOBJ::EventObj *e);
    void add_ray_traj(EVENTOBJ::EventObj *e, LinearTrajectory *trajectory, geometry_utils::Transformation *T_cw);
    int nr_ray(int x, int y, int z);
    void clear();
    void filter();
    void camInit(const sensor_msgs::CameraInfo::ConstPtr& msg);
    //void camInit(image_geometry::PinholeCameraModel& cam);

    int filtered_mark(int x, int y);
    void depth_map(cv::Mat& img);
    void npy();
    
private:

    ODOM::Position* pos;
    int dim[3];
    int FX;
    int FY;
    int CX;
    int CY;
    Eigen::Matrix3f K;


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

    image_geometry::PinholeCameraModel dvs_cam;



};
} //namespace
