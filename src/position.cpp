#include <position.hpp>
#include <iostream>

namespace ODOM{
Position::Position(ros::Publisher *odom){
    //ROS_INFO("pos init");
    this->odom = odom; 
    first = false;
   //quaterions_euler(0.8775826, 0.4794255, 0.0, 0.0);
   //ROS_INFO("rpj %f %f %f", current_pos[3], current_pos[4], current_pos[5]);
    quart = QUART::Quarternion();
    last_quart = QUART::Quarternion();
    //ROS_INFO("init done");

    hoop_pos[0] = 0;
    hoop_pos[1] = 0;
    hoop_pos[2] = 0;
}


void Position::odom_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    //ROS_INFO("odom");
/*
    latest_pos_ts = current_pos_ts;
    last_quart.update(&quart);
    for (int i = 0; i < 3; i++){
        last_pos[i] = current_pos[i];
    }

    //ROS_INFO("update last");
    // slider_depth has a non convential cordinat frame
    //current_pos[0] = msg->transform.position.z;
    //current_pos[1] = -msg->transform.position.x;
    //current_pos[2] = -msg->transform.position.y;

    //current_pos[0] = msg->transform.position.x;
    //current_pos[1] = msg->transform.position.y;
    //current_pos[2] = msg->transform.position.z;
    //quaterions_euler(msg->transform.orientation.w, 
    //                msg->transform.orientation.x, 
    //                msg->transform.orientation.y, 
    //                msg->transform.orientation.z);
    
*/
    current_pos[0] = msg->transform.translation.x;
    current_pos[1] = msg->transform.translation.y;
    current_pos[2] = msg->transform.translation.z;
    
    quart.update(msg->transform.rotation.w,
                 -msg->transform.rotation.x,
                 -msg->transform.rotation.y,
                 -msg->transform.rotation.z);
/*
    //ROS_INFO("Read new");
    if (!first){
        for(int i = 0; i < 3; i++){
            first_pos[i] = current_pos[i];
        }
        first = true;
    }
    for(int i = 0; i < 3; i++){
        current_pos[i] -= first_pos[i];
    }
    current_pos_ts = msg->header.stamp.toSec();
    translation_calk();

    //ROS_INFO("quartileon %f %f %f %f", msg->transform.orientation.w, msg->transform.orientation.x, msg->transform.orientation.y, msg->transform.orientation.z);
    //ROS_INFO("ocom_callback %f %f %f %f %f %f %f", current_pos[0], current_pos[1], current_pos[2], quart.w, quart.x, quart.y, quart.z);
    
    track.header = msg->header;


    //ROS_INFO("end pos callback");
*/
}

void Position::hula_hoop_callback( const geometry_msgs::TransformStamped::ConstPtr& msg){
/*
    latest_pos_ts = current_pos_ts;
    last_quart.update(&quart);
    for (int i = 0; i < 3; i++){
        last_pos[i] = current_pos[i];
    }
    
    hoop_pos[0] = msg->transform.translation.x;
    hoop_pos[1] = msg->transform.translation.y;
    hoop_pos[2] = msg->transform.translation.z;
*/
/*
    if (!first){
        for(int i = 0; i < 3; i++){
            first_pos[i] = current_pos[i];
        }
        first = true;
    }
    for(int i = 0; i < 3; i++){
        current_pos[i] -= first_pos[i];
    }
*/
    current_pos_ts = msg->header.stamp.toSec();

}

void Position::publish(){
    track.header.frame_id = "world";
    track.pose.position.x = my_pos[0] * 33.0 + 75;
    track.pose.position.y = my_pos[1] * 33.0 + 75;
    track.pose.position.z = my_pos[2] * 33.0 + 75;
    track.pose.orientation.x = my_pos[3];
    track.pose.orientation.y = my_pos[4];
    track.pose.orientation.z = my_pos[5];
    track.pose.orientation.w = my_pos[6];
    //track.pose.position.x = 0;// * 10 + 15;
    //track.pose.position.y = 0;// * 10 + 15;
    //track.pose.position.z = 0;// * 10 + 15;
    //track.pose.orientation.x = 0;
    //track.pose.orientation.y = 0;
    //track.pose.orientation.z = 0;
    //track.pose.orientation.w = 1;
    odom->publish(track);

}

void Position::quaterions_euler(double w, double x, double y, double z){
/*
    double test = x * y + z * w;
    double norm = pow(w,2) + pow(x,2) + pow(y,2) + pow(z,2);
    if (test > 0.499 * norm)
    {
        current_pos[4] = -0;
        current_pos[5] = -4.1415 / 2;
        current_pos[3] = 2 * std::atan2(x,w);
    }
    else if (test < -0.499 * norm){
        current_pos[4] = -0;
        current_pos[5] = 4.1415 / 2;
        current_pos[3] = -2 * std::atan2(x,w);
    }
    else {
        double sinr_cosp = 2 * (w * x - y * z);
        double cosr_cosp =(y * y - x * x - z * z + w * w);
        current_pos[4] = -std::atan2(sinr_cosp, cosr_cosp);

        current_pos[5] = -std::asin(2 * test / norm);        
        
        current_pos[3] = std::atan2(2 * (y * w - x * z), pow(x,2) - pow(y,2) - pow(z,2) + pow(w,2)); 
        
    }
    */
    /*
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp >= 1)){
        current_pos[4] = std::copysign(3.1415 / 2, sinp);
    }
    else {
        current_pos[4] = std::asin(sinp);
    }
    */
    /*
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    current_pos[5] = std::atan2(siny_cosp, cosy_cosp);
    */
    
    return;
}

void Position::translation_calk(){
    time_differens = current_pos_ts - latest_pos_ts;
    for(int i = 0; i < 3; i++){
        translation[i] = (current_pos[i] - last_pos[i]) / time_differens;
    }
    //ROS_INFO("traslaton calk %f", translation[1]);
    
    return;
}

double * Position::get_current_pos(){
    
    my_pos[0] = current_pos[0] - hoop_pos[0];
    my_pos[1] = current_pos[1] - hoop_pos[1];
    my_pos[2] = current_pos[2] - hoop_pos[2];
    my_pos[3] = quart.x;
    my_pos[4] = quart.y;
    my_pos[5] = quart.z;
    my_pos[6] = quart.w;
    publish();
    return my_pos;
    //return current_pos;
}

double * Position::pos_at(double ts){
//pos_at = (p_n * p_p^-1)^dt * p_p
// q^t = cos(t*a/2, n sin(t*a/2
// q1 * q2 = w2 * w1 - n2 * n1, w2 * n1 + w1 * n2 + n1 x n2
    //return quart;
    
    QUART::Quarternion quarternion_at = QUART::Quarternion(last_quart);
    quart.invers();
    quarternion_at.multiply(&quart);
    quarternion_at.pow(ts / time_differens);
    quart.invers();
    quarternion_at.multiply(&quart);
    double offset = ts - current_pos_ts;

    for (int i = 0; i < 3; i++){
        my_pos[i] = current_pos[i] + translation[i] * offset;
    }

    my_pos[3] = quarternion_at.x;
    my_pos[4] = quarternion_at.y;
    my_pos[5] = quarternion_at.z;
    my_pos[6] = quarternion_at.w;
    publish();
    return my_pos;
/*
    //ROS_INFO("last known %f %f %f", current_pos[0], current_pos[1], current_pos[2]);
    //ROS_INFO("delta %f %f %f", translation[0], translation[1], translation[2]);
    //ROS_INFO("my_pos %f %f %f", my_pos[0], my_pos[1], my_pos[2]);
    return my_pos;
*/
}

} //namespace
