#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/CameraInfo.h"
#include <event.hpp>
#include <voxel_grid.hpp>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <Eigen/Core>
#include <geometry_utils.hpp>
#include <trajectory.hpp>


ros::Publisher depth_img_pub;
std::map<ros::Time, geometry_utils::Transformation> poses;

// fix dynamic
//const int DIMX = 640;
//const int DIMY = 480;
const int DIMX = 240;
const int DIMY = 180;
const int DIMZ = 100;


int id = 0;
int size = 0;
double ts_offset;
bool got_cam = false;

GRID::Voxel grid(DIMX, DIMY, DIMZ); 
EVENT::Event event(&grid);


void marker(const ros::TimerEvent&){
    LinearTrajectory trajectory = LinearTrajectory(poses);
    geometry_utils::Transformation T0, T1;
    ros::Time t0, t1;
    trajectory.getFirstControlPose(&T0, &t0);
    trajectory.getLastControlPose(&T1, &t1);
    geometry_utils::Transformation T_wc;
    //trajectory.getPoseAt(ros::Time(0.5 * (t0.toSec() + t1.toSec())), T_wc);
    //trajectory.getPoseAt(ros::Time(t1.toSec()), T_wc);
    //geometry_utils::Transformation T_cw = T_wc.inverse();
    geometry_utils::Transformation T_cw = T1.inverse();

    //std::cout<<"T_cw\n"<<T_cw<<std::endl;


    event.add_ray(&trajectory, &T_cw);
    grid.filter();
    cv::Mat img(DIMY, DIMX, CV_8UC1, cv::Scalar(55));
    grid.depth_map(img);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    depth_img_pub.publish(msg);
    grid.npy();
    //exit(0);
    grid.clear();
    //poses.clear();
}

void cam_callback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    //ROS_INFO("camera");
    sensor_msgs::CameraInfo camera_info_msg = *msg;
    if (camera_info_msg.distortion_model == ""){
        camera_info_msg.distortion_model = "plumb_bob";
        // first with wornk camera setings
        camera_info_msg.D = {-0.17288558224174980,0.084240367000352243,0,0,0};
        camera_info_msg.K = 
            {533.29086448187968, 0 , 316.60334236363684, 
            0, 533.26739769548783, 209.47233667700365, 
            0, 0, 1};
        camera_info_msg.R = {1,0,0, 0,1,0, 0,0,1};
        camera_info_msg.P = 
            {533.29086448187968, 0 , 316.60334236363684, 0, 
            0, 533.26739769548783, 209.47233667700365, 0,
            0,0,1,0};
    }
    grid.camInit(&camera_info_msg);

    got_cam = true;
    return;
}


void trajectory(const Eigen::Vector3d *position, const Eigen::Quaterniond *quat, ros::Time time){
    geometry_utils::Transformation T(*position, *quat);
    poses.insert(std::pair<ros::Time, geometry_utils::Transformation>(time, T));
    return;
}

void trajectory_dvs_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    const Eigen::Vector3d position(msg->pose.position.x,
                                    msg->pose.position.y,
                                    msg->pose.position.z);
    const Eigen::Quaterniond quat(msg->pose.orientation.w,
                                msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z);
    trajectory(&position, &quat, ros::Time(msg->header.stamp.toSec()));
}

void trajectory_pro_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    const Eigen::Vector3d position(msg->transform.translation.x,
                                    msg->transform.translation.y,
                                    msg->transform.translation.z);
    const Eigen::Quaterniond quat(msg->transform.rotation.w,
                                msg->transform.rotation.x,
                                msg->transform.rotation.y,
                                msg->transform.rotation.z);
    trajectory(&position, &quat, ros::Time(msg->header.stamp.toSec()));

}



int main(int argc, char **argv){
    
    ros::init(argc, argv, "depth_map");
    ros::NodeHandle n;
    
    ros::Subscriber camera_info_dvs_sub = n.subscribe("/dvs/camera_info", 1, cam_callback);
    ros::Subscriber camera_info_pro_sub = n.subscribe("/prophesee/camera/camera_info", 1, cam_callback);
    //while(!got_cam){
    //    ros::spinOnce();
    //}

    ros::Subscriber odometry_dvs_sub = n.subscribe("/optitrack/davis", 100, trajectory_dvs_callback);
    ros::Subscriber odometry_pro_sub = n.subscribe("/vicon/event_camera/event_camera", 100, trajectory_pro_callback);

    ros::Subscriber event_dvs_sub = n.subscribe("/dvs/events", 100, &EVENT::Event::event_dvs_callback, &event);
    ros::Subscriber event_pro_sub = n.subscribe("/prophesee/camera/cd_events_buffer", 100, &EVENT::Event::event_pro_callback, &event);

    depth_img_pub = n.advertise<sensor_msgs::Image>("/event/depth_map", 1);
    ros::Timer timer = n.createTimer(ros::Duration(2), marker);


    while(ros::ok()){
        ros::spin();
    }


    return 0;
}
