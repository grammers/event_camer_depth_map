#pragma once
#include "ros/ros.h"
#include <voxel_grid.hpp>
#include <event_obj.hpp>
#include <dvs_msgs/EventArray.h>
#include <prophesee_event_msgs/EventArray.h>
#include <vector>
#include <geometry_utils.hpp>
#include <trajectory.hpp>

#define TS 0.1

namespace EVENT{

class Event{
public:
    Event(GRID::Voxel *grid);
    void event_dvs_callback(const dvs_msgs::EventArray::ConstPtr& msg);
    void event_pro_callback(const prophesee_event_msgs::EventArray::ConstPtr& msg);
    void add_ray(LinearTrajectory *trajectory, geometry_utils::Transformation *T_cw);

private:

    GRID::Voxel* grid;

    std::vector<EVENTOBJ::EventObj> event_obj;


};
}
