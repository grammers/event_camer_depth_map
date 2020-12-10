#pragma once
#include <dvs_msgs/EventArray.h>
#include <prophesee_event_msgs/EventArray.h>
#include <Eigen/Core>

namespace EVENTOBJ {
class EventObj{
public:
    EventObj(dvs_msgs::Event e);
    EventObj(prophesee_event_msgs::Event e);

    double get_ts();
    ros::Time get_ros_ts();
    float get_x();
    float get_y();

private:

    prophesee_event_msgs::Event event;

};
} // namespace
