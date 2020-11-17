#pragma once
#include "ros/ros.h"
#include <position.hpp>
#include <voxel_grid.hpp>
//#include <dvs_msgs/EventArray.h>
#include <prophesee_event_msgs/EventArray.h>

namespace EVENT{

class Event{
public:
    Event(GRID::Voxel *grid);
    void event_callback(const prophesee_event_msgs::EventArray::ConstPtr& msg);
    void set_camera(double fx, double fy, double cx, double cy);
    double get_f();
    double get_fx();
    double get_fy();

private:

    GRID::Voxel* grid;
    double f;
    double fx;
    double fy;
    double cx;
    double cy;

};
}
