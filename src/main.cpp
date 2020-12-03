#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
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

//#define FX_IN 500
//#define FY_IN 500
//#define CX 320
//#define CY 240
//#define FX_UT 220
//#define FY_UT 220
//#define WIDHT 640
//#define HEIGHT 480
#define FX_IN 199
#define FY_IN 199
#define CX 132
#define CY 114
#define WIDHT 240
#define HEIGHT 180


ros::Publisher marker_pub;

ros::Publisher odom;
ros::Publisher depth_img_pub;
visualization_msgs::MarkerArray marker_array;

const int DIMX = WIDHT;
const int DIMY = HEIGHT;
const int DIMZ = 255/3;

int width = 350;
int height = 350;

int id = 0;
int size = 0;
double ts_offset;
bool got_cam = false;

ODOM::Position pos(&odom);
//GRID::Voxel grid(10, 10, 10); 
GRID::Voxel grid(&pos, DIMX, DIMY, DIMZ, FX_IN, FY_IN, CX, CY); 
EVENT::Event event(&grid, &pos);

void depth_map(int w, int h, cv::Mat img, double* pos){
   int d = grid.filtered_mark(w,h);

    //if (d != 0){
        uchar& c = img.at<uchar>(h,w); 
        //ROS_INFO("%i", d);
        c = (uchar) d;// ((sqrt( pow(w - with/2 - pos[0],2) + pow(d - pos[1],2) + pow(h - heigh - pos[2], 2)) / 130) * 255);
    //}
}

void marker(const ros::TimerEvent&){
    event.add_ray();
    grid.filter();
    cv::Mat img(HEIGHT, WIDHT, CV_8UC1, cv::Scalar(55));
    grid.depth_map(img);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    depth_img_pub.publish(msg);
    grid.npy();
    exit(0);
    grid.clear();

/*    

    grid.filter(WIDHT, HEIGHT, FX_UT, FY_UT);


    pos.reset();
*/
}

void cam_callback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    //ROS_INFO("camera");
    width = msg->width;
    height = msg->height;
    event.set_camera(FX_IN, FY_IN, width / 2, height / 2);
    got_cam = true;
    return;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "depth_map");
    ros::NodeHandle n;
    
    event.set_camera(FX_IN, FY_IN, CX, CY);
    //ros::Subscriber camera_info_sub = n.subscribe("/dvs/camera_info", 1, cam_callback);
    //ros::Subscriber camera_info_sub = n.subscribe("/prophesee/camera/camera_info", 1, cam_callback);
    //while(!got_cam){
    //    ros::spinOnce();
    //}

    ros::Subscriber odometry_sub = n.subscribe("/optitrack/davis", 10, &ODOM::Position::odom_callback, &pos);
    ros::Subscriber event_sub = n.subscribe("/dvs/events", 10, &EVENT::Event::event_callback, &event);
    //ros::Subscriber odometry_sub = n.subscribe("/vicon/event_camera/event_camera", 10, &ODOM::Position::odom_callback, &pos);
    //ros::Subscriber event_sub = n.subscribe("/prophesee/camera/cd_events_buffer", 10, &EVENT::Event::event_callback, &event);

    depth_img_pub = n.advertise<sensor_msgs::Image>("/event/depth_map", 1);
    ros::Timer timer = n.createTimer(ros::Duration(10), marker);

    //debug calibrations
    ros::Subscriber odometry_hoop_sub = n.subscribe("/vicon/hoop/hoop", 10, &ODOM::Position::hula_hoop_callback, &pos);
    odom = n.advertise<geometry_msgs::PoseStamped>("/optitrack/map",10);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>( "/visualization_marker", 0 );



    while(ros::ok()){
        ros::spin();
    }


    return 0;
}
