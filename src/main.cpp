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

#define FX_IN 300
#define FY_IN 300
#define FX_UT 220
#define FY_UT 220
#define WIDHT 256
#define HEIGHT 256


ros::Publisher marker_pub;

ros::Publisher odom;
ros::Publisher depth_img_pub;
visualization_msgs::MarkerArray marker_array;

const int DIMX = 110;
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
GRID::Voxel grid(&pos, DIMX, DIMY, DIMZ); 
EVENT::Event event(&grid);

void depth_map(int w, int h, cv::Mat img, double* pos){
   int d = grid.filtered_mark(w,h);

    //if (d != 0){
        uchar& c = img.at<uchar>(h,w); 
        //ROS_INFO("%i", d);
        c = (uchar) d;// ((sqrt( pow(w - with/2 - pos[0],2) + pow(d - pos[1],2) + pow(h - heigh - pos[2], 2)) / 130) * 255);
    //}
}

void marker(const ros::TimerEvent&){
    
    cv::Mat img(HEIGHT, WIDHT, CV_8UC1, cv::Scalar(255));

    grid.filter(WIDHT, HEIGHT, FX_UT, FY_UT);
    grid.depth_map(img);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    depth_img_pub.publish(msg);

    grid.clear();
    pos.reset();
}

void cam_callback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    //ROS_INFO("camera");
    width = msg->width;
    height = msg->height;
    event.set_camera(FX_IN, FY_UT, width / 2, height / 2);
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
    ros::Subscriber event_sub = n.subscribe("/prophesee/camera/cd_events_buffer", 10, &EVENT::Event::event_callback, &event);

    depth_img_pub = n.advertise<sensor_msgs::Image>("/event/depth_map", 1);
    ros::Timer timer = n.createTimer(ros::Duration(1), marker);

    //debug calibrations
    ros::Subscriber odometry_hoop_sub = n.subscribe("/vicon/hoop/hoop", 10, &ODOM::Position::hula_hoop_callback, &pos);
    odom = n.advertise<geometry_msgs::PoseStamped>("/optitrack/map",10);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>( "/visualization_marker", 0 );



    while(ros::ok()){
        ros::spin();
    }


    return 0;
}
