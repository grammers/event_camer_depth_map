#pragma once
#include "geometry_msgs/PoseStamped.h"
//#include <ros/console.h>
#include "ros/ros.h"
#include <cmath>

namespace ODOM{

class Position{
public:
    Position(ros::Publisher *odom);
    void odom_callback(const geometry_msgs::PoseStamped::ConstPtr& mag);
    double * pos_at(double ts);
    double * get_current_pos();

private:
    ros::Publisher* odom;
    
    double current_pos [6];
    double quart [7];
    double last_pos [6];
    double first_pos [6];
    double latest_pos_ts;
    double current_pos_ts;
    double my_pos [6];
    
    double translation [6];

    bool first;

    void quaterions_euler(double w, double x, double y, double z);
    void translation_calk();
};
}
