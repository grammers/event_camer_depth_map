#include "ros/ros.h"
//#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/CameraInfo.h"
#include <position.hpp>
#include <event.hpp>
#include <voxel_grid.hpp>



ODOM::Position pos;
GRID::Voxel grid(100, 100, 100); // 1dm presition -> 5m eatch direction
EVENT::Event event(pos, &grid);

void cam_callback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    event.set_camera(msg->K[0], msg->K[4], msg->K[2], msg->K[5]);
    return;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "depth_map");
    ros::NodeHandle n;
    
    ros::Subscriber camera_info_sub = n.subscribe("/dvs/camer_info", 1, cam_callback);

    ros::Subscriber odometry_sub = n.subscribe("/optitrack/davis", 10, &ODOM::Position::odom_callback, &pos);

    ros::Subscriber event_sub = n.subscribe("/dvs/events", 10, &EVENT::Event::event_callback, &event);

    while(ros::ok()){
        ros::spin();
    }


    return 0;
}
