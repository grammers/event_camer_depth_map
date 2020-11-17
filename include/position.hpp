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

    double * get_current_pos();
    void dir_vector(double *dir);
    void reset();

private:
    ros::Publisher* odom;
    geometry_msgs::PoseStamped track;
    
    double pos_origin [3];
    double pos_last [3];
    double pos_relative [3];
    double hoop_pos[3];

    QUAT::Quaternion quat_origin;
    QUAT::Quaternion quat_last;
    QUAT::Quaternion quat_relative;

    double my_pos [7];
    
    void publish();
};
}
