#pragma once
#include <dvs_msgs/EventArray.h>
//#include <prophesee_event_msgs/EventArray.h>
#include <position.hpp>
#include <Eigen/Core>

namespace EVENTOBJ {
class EventObj{
public:
    EventObj(dvs_msgs::Event e, ODOM::Position *pos);
    //EventObj(prophesee_event_msgs::Event e, ODOM::Position *pos);

    double get_ts();
    ros::Time get_ros_ts();
    float get_x();
    float get_y();
    Eigen::Vector3f get_pos();
    Eigen::Matrix3f get_rot();

private:

    dvs_msgs::Event event;
    //prophesee_event_msgs::Event event;
    //ODOM::Position *pos;
    Eigen::Vector3f v_pos;
    Eigen::Matrix3f r_pos;


};
} // namespace
