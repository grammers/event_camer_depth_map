#include "ros/ros.h"
//#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/CameraInfo.h"
#include "image_transport/image_transport.h"
#include <position.hpp>
#include <event.hpp>
#include <voxel_grid.hpp>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

//#define CV_8UC1 CV_MAKETYPE(CV_8U,1)
ros::Publisher depth_img_pub;

int width = 350;
int height = 350;

double ts_offset;

ODOM::Position pos;
//GRID::Voxel grid(10, 10, 10); 
GRID::Voxel grid(300, 300, 300); 
EVENT::Event event(&pos, &grid);

void cam_callback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    event.set_camera(msg->K[0], msg->K[4], msg->K[2], msg->K[5]);
    width = msg->width;
    height = msg->height;
    //ts_offset = msg->header.stamp.toSec();
    ts_offset = ros::Time::now().toSec();
    return;
}

void depthMap(const ros::TimerEvent&){
    ROS_INFO("time");
    double* camera_pos = pos.get_current_pos(); //pos.pos_at(ros::Time::now().toSec() - ts_offset);
    double pixel_vector [3];
    //ROS_INFO("Cam pos %f %f %f",camera_pos[0], camera_pos[1], camera_pos[2]);


    //pixel_vector[0] = 10.0;
    //pixel_vector[1] = 10.0;
    pixel_vector[2] = event.get_f() /2;
    //double pixle_value = grid.depth_at_pixel(camera_pos, pixel_vector);
    //ROS_INFO("new map");    
    cv::Mat image(height, width, CV_8UC1, cv::Scalar(1));

    //ROS_INFO("loop");
    
    double min = 1000;
    double max = 0;
    for(int y=0; y < height; y++){
        //uchar* row = image.ptr<uchar>(y);
        for(int x = 0; x < width; x++){
            pixel_vector[0] = x - width / 2;
            pixel_vector[1] = y - height / 2;
            //ROS_INFO("vec %f %f", pixel_vector[0], pixel_vector[1]);
                
            double pixle_value = grid.depth_at_pixel(camera_pos, pixel_vector);
            
            uchar color = (uchar) (((pixle_value - 30)/ (20)) * 255);
            
            //row[x] = color;
            
            if (pixle_value != 300.0){
                if (pixle_value < min){ min = pixle_value;}
                if (pixle_value > max){ max = pixle_value;}
                //ROS_INFO("pixel value %f", pixle_value);
                //ROS_INFO("color %i", color);
                //ROS_INFO("cord %i %i", x, y);
                //ROS_INFO("vec %f %f", pixel_vector[0], pixel_vector[1]);
                //ROS_INFO("size %i %i", width, height);
                uchar& c = image.at<uchar>(y, x);
                c = color;
            }
            //ROS_INFO("color %i", color);
            //image.at<uchar>(y,x) = color;
            //uchar r = image.at<uchar>(y,x);
            //ROS_INFO("color new %i", r);
        }
    }

    ROS_INFO("min max %f %f", min, max);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    depth_img_pub.publish(msg);
    //image.release();
    //ROS_INFO("post");

}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "depth_map");
    ros::NodeHandle n;
    
    ros::Subscriber camera_info_sub = n.subscribe("/dvs/camera_info", 1, cam_callback);

    ros::Subscriber odometry_sub = n.subscribe("/optitrack/davis", 10, &ODOM::Position::odom_callback, &pos);

    ros::Subscriber event_sub = n.subscribe("/dvs/events", 10, &EVENT::Event::event_callback, &event);

    ros::Timer timer = n.createTimer(ros::Duration(1.0), depthMap);
    //image_transport::ImageTransport it(n);
    depth_img_pub = n.advertise<sensor_msgs::Image>("/event/depth_map", 1);

    while(ros::ok()){
        ros::spin();
    }


    return 0;
}
