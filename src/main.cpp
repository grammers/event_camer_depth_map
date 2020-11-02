#include "ros/ros.h"
//#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/CameraInfo.h"
#include "visualization_msgs/MarkerArray.h"
#include <position.hpp>
#include <event.hpp>
#include <voxel_grid.hpp>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <rviz_visual_tools/rviz_visual_tools.h>

//#define CV_8UC1 CV_MAKETYPE(CV_8U,1)
ros::Publisher marker_pub;

ros::Publisher odom;
visualization_msgs::MarkerArray marker_array;

const int DIMX = 100;
const int DIMY = 100;
const int DIMZ = 100;

int width = 350;
int height = 350;

int id = 0;
int size = 0;
double ts_offset;

ODOM::Position pos(&odom);
//GRID::Voxel grid(10, 10, 10); 
GRID::Voxel grid(DIMX, DIMY, DIMZ); 
EVENT::Event event(&pos, &grid);


//void add_marker(visualization_msgs::MarkerArray *marker_array, int x, int y, int z){
void add_marker(int x, int y, int z){
    if (grid.is_marked(x, y, z)){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        //ROS_INFO("MARKER %i %i %i", x ,y ,z);
        marker_array.markers.push_back(marker);
        size++;
    }

}

void addaptiv(int w, int h){
    int d = grid.filtered_mark(w, h);
    if (d >= 0){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = d;
        marker.pose.position.y = w;
        marker.pose.position.z = h;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //ROS_INFO("MARKER %i %i %i", x ,y ,z);
        marker_array.markers.push_back(marker);
        size++;
    }

}

void add_2d_marker(int w, int h){
    int max = 0;
    int D = 0;
    for (int d = 0; d < DIMX; d++){
        if (grid.nr_ray(w, h, d) > max){
            max = grid.nr_ray(w, h, d);
            D = d;
        }
    }
    if (max > 1000){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = w;
        marker.pose.position.y = h;
        marker.pose.position.z = D;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //ROS_INFO("MARKER %i %i %i", x ,y ,z);
        marker_array.markers.push_back(marker);
        size++;

    }
}

void marker(const ros::TimerEvent&){
    //rviz_visual_tools::RvizVisualTools rviz();
    //rviz->deleteAllMarkers();
    //ROS_INFO("WHER");
    for (int i = 0; i < marker_array.markers.size(); i++){
        marker_array.markers[i].header.stamp = ros::Time();
        marker_array.markers[i].action = 2;
    }
    //marker_pub.publish(marker_array);
    marker_array.markers.empty();
    
    double fw = event.get_fx();
    double fh = event.get_fy();
    double* position = pos.get_current_pos();

    grid.filter(position, width, height,fw,fh);
    size = 0;
    for (int w = 0; w < width; w++){ 
        for (int h = 0; h < height; h++){
            addaptiv(w, h);
            //add_2d_marker(x, y);
        }
    }
    grid.normalise();
    for (int x = 0; x < DIMX; x++){ 
        for (int y = 0; y < DIMY; y++){
            for (int z = 0; z < DIMZ; z++){
                add_marker(x, y, z);
            }
        }
    }
    //ROS_INFO("re pub");
    marker_pub.publish(marker_array);
}

void cam_callback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    event.set_camera(msg->K[0], msg->K[4], msg->K[2], msg->K[5]);
    width = msg->width;
    height = msg->height;
    //ts_offset = msg->header.stamp.toSec();
    ts_offset = ros::Time::now().toSec();
    return;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "depth_map");
    ros::NodeHandle n;
    
    ros::Subscriber camera_info_sub = n.subscribe("/dvs/camera_info", 1, cam_callback);

    ros::Subscriber odometry_sub = n.subscribe("/optitrack/davis", 10, &ODOM::Position::odom_callback, &pos);
    odom = n.advertise<geometry_msgs::PoseStamped>("/optitrack/map",10);

    ros::Subscriber event_sub = n.subscribe("/dvs/events", 10, &EVENT::Event::event_callback, &event);

    marker_pub = n.advertise<visualization_msgs::MarkerArray>( "/visualization_marker", 0 );
    ros::Timer timer = n.createTimer(ros::Duration(1.5), marker);


    while(ros::ok()){
        ros::spin();
    }


    return 0;
}
