#include <position.hpp>
#include <iostream>

namespace ODOM{
//Position::Position(){
//    std::cout<<"position"<<std::endl;
//    ROS_INFO("POSITION ");
//}


void Position::odom_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    latest_pos_ts = current_pos_ts;
    for(int i = 0; i < 6; i++){
        last_pos[i] = current_pos[i];
    }

    // slider_depth has a non convential cordinat frame
    current_pos[0] = msg->pose.position.y;
    current_pos[1] = -msg->pose.position.x;

    //current_pos[0] = msg->pose.position.x;
    //current_pos[1] = msg->pose.position.y;
    current_pos[2] = msg->pose.position.z;
    quaterions_euler(msg->pose.orientation.w, 
                    msg->pose.orientation.x, 
                    msg->pose.orientation.y, 
                    msg->pose.orientation.z);
    
    current_pos_ts = msg->header.stamp.toSec();
    translation_calk();

    ROS_INFO("ocom_callback %f", current_pos[1]);
}

void Position::quaterions_euler(double w, double x, double y, double z){
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 1 * (x * x + y * y);
    current_pos[3] = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (w * y + z * x);
    if (std::abs(sinp >= 1)){
        current_pos[4] = std::copysign(3.1415 / 2, sinp);
    }
    else {
        current_pos[4] = std::asin(sinp);
    }

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    current_pos[5] = std::atan2(siny_cosp, cosy_cosp);

    
    return;
}

void Position::translation_calk(){
    double diff = current_pos_ts - latest_pos_ts;
    for(int i = 0; i < 6; i++){
        translation[i] = (current_pos[i] - last_pos[i]) / diff;
    }
    //ROS_INFO("traslaton calk %f", translation[1]);
    
    return;
}

double * Position::get_current_pos(){
    return current_pos;
}

double * Position::pos_at(double ts){
    double offset = ts - current_pos_ts;

    for (int i = 0; i < 6; i++){
        my_pos[i] = current_pos[i] + translation[i] * offset;
    }

    //ROS_INFO("last known %f %f %f", current_pos[0], current_pos[1], current_pos[2]);
    //ROS_INFO("delta %f %f %f", translation[0], translation[1], translation[2]);
    //ROS_INFO("my_pos %f %f %f", my_pos[0], my_pos[1], my_pos[2]);
    return my_pos;
}

} //namespace
