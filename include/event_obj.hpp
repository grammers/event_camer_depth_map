#pragma once
#include <prophesee_event_msgs/EventArray.h>
#include <position.hpp>
#include <Eigen/Core>

namespace EVENTOBJ {
class EventObj{
public:
    EventObj(prophesee_event_msgs::Event e, ODOM::Position *pos);

    int get_ts();
    float get_x();
    float get_y();
    Eigen::Vector3f get_pos();
    Eigen::Matrix3f get_rot();

private:

    prophesee_event_msgs::Event event;
    //ODOM::Position *pos;
    Eigen::Vector3f v_pos;
    Eigen::Matrix3f r_pos;


};
} // namespace
