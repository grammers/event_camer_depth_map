#pragma once
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "quaternion.hpp"
//#include <ros/console.h>
#include "ros/ros.h"
#include <cmath>
#include <Eigen/Core>

namespace ODOM{

class Position{
public:
    Position(ros::Publisher *odom);
    Position(Position *position);

    void odom_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void hula_hoop_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);

    Eigen::Vector3f get_current_pos();
    Eigen::Matrix3f get_rot();
    void dir_vector(double *dir);
    //void reset();

private:
    ros::Publisher* odom;
    geometry_msgs::PoseStamped track;
    
    double pos [3];
    //double pos_last [3];
    //double pos_relative [3];
    //double hoop_pos[3];

    QUAT::Quaternion quat;
    //QUAT::Quaternion quat_last;
    //QUAT::Quaternion quat_relative;

    //double my_pos [7];
    
    void publish();
};
}
