#pragma once
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "quaternion.hpp"
//#include <ros/console.h>
#include "ros/ros.h"
#include <cmath>

namespace ODOM{

class Position{
public:
    Position(ros::Publisher *odom);
    void odom_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void hula_hoop_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
    double * pos_at(double ts);
    double * get_current_pos();

private:
    ros::Publisher* odom;
    geometry_msgs::PoseStamped track;
    
    double current_pos [7];
    double hoop_pos[3];
    QUART::Quarternion quart;
    QUART::Quarternion last_quart;
    double last_pos [3];
    double first_pos [3];
    double latest_pos_ts;
    double current_pos_ts;
    double time_differens;
    double my_pos [7];
    
    double translation [3];

    bool first;

    void quaterions_euler(double w, double x, double y, double z);
    void translation_calk();
    void publish();
};
}
