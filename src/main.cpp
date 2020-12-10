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
#include <Eigen/Core>
#include <geometry_utils.hpp>
#include <trajectory.hpp>

/*
#define FX_IN 199.0923665423112
#define FY_IN 198.8288204700886
#define CX 132.192071377002
#define CY 110.7126600112956
//#define FX_UT 220
//#define FY_UT 220
#define WIDHT 240
#define HEIGHT 180
*/
#define FX_IN 531.59205887813710
#define FY_IN 531.77560077034229
#define CX 316.22939110050925
#define CY 209.21994067113124
#define WIDHT 640
#define HEIGHT 480


ros::Publisher marker_pub;

ros::Publisher odom;
ros::Publisher depth_img_pub;
visualization_msgs::MarkerArray marker_array;
std::map<ros::Time, geometry_utils::Transformation> poses;

const int DIMX = WIDHT;
const int DIMY = HEIGHT;
const int DIMZ = 100;

int width = WIDHT;
int height = HEIGHT;

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
    LinearTrajectory trajectory = LinearTrajectory(poses);
    geometry_utils::Transformation T0, T1;
    ros::Time t0, t1;
    trajectory.getFirstControlPose(&T0, &t0);
    trajectory.getLastControlPose(&T1, &t1);
    geometry_utils::Transformation T_wc;
    trajectory.getPoseAt(ros::Time(0.5 * (t0.toSec() + t1.toSec())), T_wc);
    //trajectory.getPoseAt(ros::Time(0.9 * (t0.toSec() + t1.toSec())), T_wc);
    geometry_utils::Transformation T_cw = T_wc.inverse();

    //std::cout<<"T_cw\n"<<T_cw<<std::endl;


    event.add_ray(&trajectory, &T_cw);
    grid.filter();
    cv::Mat img(HEIGHT, WIDHT, CV_8UC1, cv::Scalar(55));
    grid.depth_map(img);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    depth_img_pub.publish(msg);
    grid.npy();
    //exit(0);
    grid.clear();
    poses.clear();

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

    //image_geometry::PinholeCameraModel cam;
    //cam.fromCameraInfo(msg);
    sensor_msgs::CameraInfo camera_info_msg = *msg;
    if (camera_info_msg.distortion_model == ""){
        camera_info_msg.distortion_model = "plumb_bob";
        // first with wornk camera setings
        camera_info_msg.D = {-0.17056838404377869,0.083318889979031738,0,0,0};
        camera_info_msg.K = {531.59205887813710, 0 , 316.22939110050925, 0, 531.77560077034229, 209.21994067113124, 0, 0, 1};
        camera_info_msg.R = {1,0,0, 0,1,0, 0,0,1};
        camera_info_msg.P = {531.59205887813710,0,316.22939110050925,0, 0,531.77560077034229,209.21994067113124,0 ,0,0,1,0};
    }
    grid.camInit(&camera_info_msg);

    got_cam = true;
    return;
}

//void trajectory_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
void trajectory_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
/*
    const Eigen::Vector3d position(msg->pose.position.x,
                                    msg->pose.position.y,
                                    msg->pose.position.z);
    const Eigen::Quaterniond quat(msg->pose.orientation.w,
                                msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z);
    */
    const Eigen::Vector3d position(msg->transform.translation.x,
                                    msg->transform.translation.y,
                                    msg->transform.translation.z);
    const Eigen::Quaterniond quat(msg->transform.rotation.w,
                                msg->transform.rotation.x,
                                msg->transform.rotation.y,
                                msg->transform.rotation.z);

    geometry_utils::Transformation T(position, quat);
    poses.insert(std::pair<ros::Time, geometry_utils::Transformation>(ros::Time(msg->header.stamp.toSec()), T));
    return;
}



int main(int argc, char **argv){
    
    ros::init(argc, argv, "depth_map");
    ros::NodeHandle n;
    
    event.set_camera(FX_IN, FY_IN, CX, CY);
    ros::Subscriber camera_info_sub = n.subscribe("/dvs/camera_info", 1, cam_callback);
    ros::Subscriber camera_info_sub = n.subscribe("/prophesee/camera/camera_info", 1, cam_callback);
    //while(!got_cam){
    //    ros::spinOnce();
    //}

    //ros::Subscriber odometry_sub = n.subscribe("/optitrack/davis", 10, &ODOM::Position::odom_callback, &pos);
    //ros::Subscriber odometry_sub = n.subscribe("/optitrack/davis", 100, trajectory_callback);
    ros::Subscriber odometry_sub = n.subscribe("/vicon/event_camera/event_camera", 100, trajectory_callback);
    //ros::Subscriber event_sub = n.subscribe("/dvs/events", 100, &EVENT::Event::event_callback, &event);
    //ros::Subscriber odometry_sub = n.subscribe("/vicon/event_camera/event_camera", 10, &ODOM::Position::odom_callback, &pos);
    ros::Subscriber event_sub = n.subscribe("/prophesee/camera/cd_events_buffer", 100, &EVENT::Event::event_callback, &event);

    depth_img_pub = n.advertise<sensor_msgs::Image>("/event/depth_map", 1);
    ros::Timer timer = n.createTimer(ros::Duration(2), marker);

    //debug calibrations
    //ros::Subscriber odometry_hoop_sub = n.subscribe("/vicon/hoop/hoop", 10, &ODOM::Position::hula_hoop_callback, &pos);
    odom = n.advertise<geometry_msgs::PoseStamped>("/optitrack/map",10);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>( "/visualization_marker", 0 );



    while(ros::ok()){
        ros::spin();
    }


    return 0;
}
