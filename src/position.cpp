#include <position.hpp>
#include <iostream>

namespace ODOM{
Position::Position(ros::Publisher *odom){
    this->odom = odom; 
    quat = QUAT::Quaternion();
    //quat_last = QUAT::Quaternion();
    //quat_relative = QUAT::Quaternion(&quat_origin, &quat_last);

    memset(pos, 0, 3);
    //memset(pos_last, 0, 3);
    //memset(pos_relative, 0, 3);
    //memset(hoop_pos, 0, 3);
}

Position::Position(Position *position) : Position(position->odom){
    this->quat = QUAT::Quaternion(position->quat);
    for(int i = 0; i < 3; i++){
        pos[i] = position->pos[i]+1;
    }
    //std::cout<<"r\n"<<quat.r<<std::endl;
}

/*
void Position::reset(){
    quat_origin.update(&quat_last);
    quat_relative = QUAT::Quaternion(&quat_origin);

    for (int i = 0; i < 3; i++){
        pos_origin[i] = pos_last[i];
        pos_relative[i] = 0;
    }

}
*/

void Position::odom_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    
    pos[0] = msg->transform.translation.x;
    pos[1] = msg->transform.translation.y;
    pos[2] = msg->transform.translation.z;
    
    quat.update(msg->transform.rotation.w,
                 msg->transform.rotation.x,
                 msg->transform.rotation.y,
                 msg->transform.rotation.z);

    //quat_relative = QUAT::Quaternion(&quat_origin, &quat_last);

    //quat_origin.translate(pos_relative, pos_origin, pos_last);

    //ROS_INFO("origin %f %f %f %f %f %f %f", pos_origin[0], pos_origin[1], pos_origin[2], quat_origin.x, quat_origin.y, quat_origin.z, quat_origin.w);
    //ROS_INFO("last %f %f %f %f %f %f %f", pos_last[0], pos_last[1], pos_last[2], quat_last.x, quat_last.y, quat_last.z, quat_last.w);
    //ROS_INFO("relativ %f %f %f %f %f %f %f", pos_relative[0], pos_relative[1], pos_relative[2], quat_relative.x, quat_relative.y, quat_relative.z, quat_relative.w);
}

void Position::hula_hoop_callback( const geometry_msgs::TransformStamped::ConstPtr& msg){
    /*
    ROS_INFO("GT dist %f", sqrt(
        POW(msg->transform.translation.x - pos_last[0]) +
        POW(msg->transform.translation.y - pos_last[1]) +
        POW(msg->transform.translation.z - pos_last[2])
        ));
    */

}
/*
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
*/
 Eigen::Vector3f Position::get_current_pos(){
    
    Eigen::Vector3f position;
    position << pos[0], pos[1], pos[2];
    //std::cout<<"pos pos\n"<<position<<std::endl;
    return position;
}

Eigen::Matrix3f Position::get_rot(){
    //std::cout<<"r\n"<<quat.r<<std::endl;
    //ROS_INFO("retrun rot");
    return quat.r;
}

void Position::dir_vector(double *dir){
    quat.rotate(dir);
}


} //namespace
