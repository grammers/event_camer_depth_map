#pragma once
#include "ros/ros.h"
#include <position.hpp>
#include <voxel_grid.hpp>
#include <event_obj.hpp>
//#include <dvs_msgs/EventArray.h>
#include <prophesee_event_msgs/EventArray.h>
#include <vector>

#define TS 0.1

namespace EVENT{

class Event{
public:
    Event(GRID::Voxel *grid, ODOM::Position *pos);
    void event_callback(const prophesee_event_msgs::EventArray::ConstPtr& msg);
    void set_camera(double fx, double fy, double cx, double cy);
    void add_ray();
    double get_f();
    double get_fx();
    double get_fy();

private:

    GRID::Voxel* grid;
    ODOM::Position* pos;

    std::vector<EVENTOBJ::EventObj> event_obj;

    void dropout();

    double f;
    double fx;
    double fy;
    double cx;
    double cy;

};
}
