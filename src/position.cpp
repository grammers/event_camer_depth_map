#include <position.hpp>
#include <iostream>

namespace ODOM{
Position::Position(ros::Publisher *odom){
    this->odom = odom; 
    quat_origin = QUAT::Quaternion();
    quat_last = QUAT::Quaternion();
    quat_relative = QUAT::Quaternion(&quat_origin, &quat_last);

    memset(pos_origin, 0, 3);
    memset(pos_last, 0, 3);
    memset(pos_relative, 0, 3);
    memset(hoop_pos, 0, 3);
}

void Position::reset(){
    quat_origin.update(&quat_last);
    quat_relative = QUAT::Quaternion(&quat_origin);

    for (int i = 0; i < 3; i++){
        pos_origin[i] = pos_last[i];
        pos_relative[i] = 0;
    }

}

void Position::odom_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    
    pos_last[0] = msg->transform.translation.x;
    pos_last[1] = msg->transform.translation.y;
    pos_last[2] = msg->transform.translation.z;
    
    quat_last.update(msg->transform.rotation.w,
                 msg->transform.rotation.x,
                 msg->transform.rotation.y,
                 msg->transform.rotation.z);

    quat_relative = QUAT::Quaternion(&quat_origin, &quat_last);

    quat_origin.translate(pos_relative, pos_origin, pos_last);

    //ROS_INFO("origin %f %f %f %f %f %f %f", pos_origin[0], pos_origin[1], pos_origin[2], quat_origin.x, quat_origin.y, quat_origin.z, quat_origin.w);
    //ROS_INFO("last %f %f %f %f %f %f %f", pos_last[0], pos_last[1], pos_last[2], quat_last.x, quat_last.y, quat_last.z, quat_last.w);
    //ROS_INFO("relativ %f %f %f %f %f %f %f", pos_relative[0], pos_relative[1], pos_relative[2], quat_relative.x, quat_relative.y, quat_relative.z, quat_relative.w);
}

void Position::hula_hoop_callback( const geometry_msgs::TransformStamped::ConstPtr& msg){
    ROS_INFO("GT dist %f", sqrt(
        POW(msg->transform.translation.x - pos_last[0]) +
        POW(msg->transform.translation.y - pos_last[1]) +
        POW(msg->transform.translation.z - pos_last[2])
        ));

}

void Position::publish(){
    track.header.frame_id = "world";
    track.pose.position.x = my_pos[0];// * 33.0 + 75;
    track.pose.position.y = my_pos[1];// * 33.0 + 75;
    track.pose.position.z = my_pos[2];// * 33.0 + 75;
    track.pose.orientation.x = my_pos[3];
    track.pose.orientation.y = my_pos[4];
    track.pose.orientation.z = my_pos[5];
    track.pose.orientation.w = my_pos[6];
    odom->publish(track);

}

double * Position::get_current_pos(){
    
    my_pos[0] = pos_relative[0];
    my_pos[1] = pos_relative[1];
    my_pos[2] = pos_relative[2];
    my_pos[3] = quat_relative.x;
    my_pos[4] = quat_relative.y;
    my_pos[5] = quat_relative.z;
    my_pos[6] = quat_relative.w;
    //publish();
    return my_pos;
}

void Position::dir_vector(double *dir){
    quat_relative.rotate(dir);
}


} //namespace
