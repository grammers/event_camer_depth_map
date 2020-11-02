#pragma once
#include "ros/ros.h"
#include <position.hpp>
#include <voxel_grid.hpp>
#include <dvs_msgs/EventArray.h>

namespace EVENT{

class Event{
public:
    Event(ODOM::Position *p, GRID::Voxel *grid);
    void event_callback(const dvs_msgs::EventArray::ConstPtr& msg);
    void set_camera(double fx, double fy, double cx, double cy);
    double get_f();
    double get_fx();
    double get_fy();

private:

    GRID::Voxel* grid;
    ODOM::Position* pos;
    double f;
    double fx;
    double fy;
    double cx;
    double cy;

};
}
