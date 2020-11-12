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
ros::Publisher depth_img_pub;
visualization_msgs::MarkerArray marker_array;

const int DIMX = 150;
const int DIMY = 150;
const int DIMZ = 150;

int width = 350;
int height = 350;

int id = 0;
int size = 0;
double ts_offset;
bool got_cam = false;

ODOM::Position pos(&odom);
//GRID::Voxel grid(10, 10, 10); 
GRID::Voxel grid(DIMX, DIMY, DIMZ); 
EVENT::Event event(&pos, &grid);


//void add_marker(visualization_msgs::MarkerArray *marker_array, int x, int y, int z){
void add_marker(int x, int y, int z){
    if (grid.is_marked(x, y, z)){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
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
    if (d > 0){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = w;
        marker.pose.position.y = d;
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
        marker.header.frame_id = "world";
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

void depth_map(int w, int h, cv::Mat img, double* pos){
   int d = grid.filtered_mark(w,h);

    //if (d != 0){
        uchar& c = img.at<uchar>(h,w); 
        //ROS_INFO("%i", d);
        c = (uchar) d;// ((sqrt( pow(w - with/2 - pos[0],2) + pow(d - pos[1],2) + pow(h - heigh - pos[2], 2)) / 130) * 255);
    //}
}

void marker(const ros::TimerEvent&){
    //rviz_visual_tools::RvizVisualTools rviz();
    //rviz->deleteAllMarkers();
    //ROS_INFO("WHER");
    int width_ = width;// / 2;
    int height_ = height;// /2;
    /*
    for (int i = 0; i < marker_array.markers.size(); i++){
        marker_array.markers[i].header.stamp = ros::Time();
        marker_array.markers[i].action = 2;
    }
    //marker_pub.publish(marker_array);
    marker_array.markers.empty();
   */ 
    
    double fw = event.get_fx();// / 3;
    double fh = event.get_fy();// / 3;
    double* position = pos.get_current_pos();

    cv::Mat img(height_, width_, CV_8UC1, cv::Scalar(1));

    //ROS_INFO("setup");
    grid.filter(position, width_, height_,fw,fh);
    size = 0;
    //ROS_INFO("filtered");
    for (int w = 0; w < width_; w++){ 
        for (int h = 0; h < height_; h++){
            //addaptiv(w, h);
            //add_2d_marker(w, h);
            depth_map(w, h, img, position);
        }
    }
    
    /*
    //ROS_INFO("betwen");
    grid.normalise();
    for (int x = 0; x < DIMX; x++){ 
        for (int y = 0; y < DIMY; y++){
            for (int z = 0; z < DIMZ; z++){
                add_marker(x, y, z);
            }
        }
    }
    */
    
    //ROS_INFO("re pub");
    //marker_pub.publish(marker_array);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    depth_img_pub.publish(msg);
}

void cam_callback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    //ROS_INFO("camera");
    //event.set_camera(msg->K[0], msg->K[4], msg->K[2], msg->K[5]);
    event.set_camera(224, 224, 320, 240);
    width = msg->width;
    height = msg->height;
    //ts_offset = msg->header.stamp.toSec();
    ts_offset = ros::Time::now().toSec();
    got_cam = true;
    return;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "depth_map");
    ros::NodeHandle n;
    
    ros::Subscriber camera_info_sub = n.subscribe("/prophesee/camera/camera_info", 1, cam_callback);
    while(!got_cam){
        ros::spinOnce();
    }

    ros::Subscriber odometry_sub = n.subscribe("/vicon/event_camera/event_camera", 10, &ODOM::Position::odom_callback, &pos);
    ros::Subscriber odometry_hoop_sub = n.subscribe("/vicon/hoop/hoop", 10, &ODOM::Position::hula_hoop_callback, &pos);
    odom = n.advertise<geometry_msgs::PoseStamped>("/optitrack/map",10);

    ros::Subscriber event_sub = n.subscribe("/prophesee/camera/cd_events_buffer", 10, &EVENT::Event::event_callback, &event);

    marker_pub = n.advertise<visualization_msgs::MarkerArray>( "/visualization_marker", 0 );
    depth_img_pub = n.advertise<sensor_msgs::Image>("/event/depth_map", 1);
    ros::Timer timer = n.createTimer(ros::Duration(0.5), marker);


    while(ros::ok()){
        ros::spin();
    }


    return 0;
}
