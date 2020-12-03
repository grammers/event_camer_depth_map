#include <position.hpp>
#include <iostream>

namespace ODOM{
Position::Position(ros::Publisher *odom){
    this->odom = odom; 
    quat_current = QUAT::Quaternion();
    quat_past = QUAT::Quaternion();
    //quat_last = QUAT::Quaternion();
    //quat_relative = QUAT::Quaternion(&quat_origin, &quat_last);

    memset(pos_current, 0, 3);
    memset(pos_past, 0, 3);
    //memset(pos_last, 0, 3);
    //memset(pos_relative, 0, 3);
    //memset(hoop_pos, 0, 3);
    dt = 0;
    t_past = 0;
    t_current = 0;
}

Position::Position(Position *position) : Position(position->odom){
    this->quat_current = QUAT::Quaternion(position->quat_current);
    for(int i = 0; i < 3; i++){
        pos_current[i] = position->pos_current[i]+1;
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

void Position::odom_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
//void Position::odom_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    if(t_current == 0)
        t_current = msg->header.stamp.toSec();
    
    t_past = t_current;
    quat_past.update(&quat_current);
    
    for(int i = 0; i < 3; i++){
        pos_past[i] = pos_current[i];
    }

    t_current = msg->header.stamp.toSec();
    
    dt = t_current - t_past;

    pos_current[0] = msg->pose.position.x;
    pos_current[1] = msg->pose.position.y;
    pos_current[2] = msg->pose.position.z;
    
    quat_current.update(msg->pose.orientation.w,
                 msg->pose.orientation.x,
                 msg->pose.orientation.y,
                 msg->pose.orientation.z);

   //std::cout<<"pos in \t"<<pos_current[0]<< "\t"<<pos_current[1]<<"\t"<<pos_current[2]<<std::endl; 
   //std::cout<<"pos past \t"<<pos_past[0]<< "\t"<<pos_past[1]<<"\t"<<pos_past[2]<<std::endl; 

    //pos[0] = msg->transform.translation.x;
    //pos[1] = msg->transform.translation.y;
    //pos[2] = msg->transform.translation.z;
    
    //quat.update(msg->transform.rotation.w,
    //             msg->transform.rotation.x,
    //             msg->transform.rotation.y,
    //             msg->transform.rotation.z);

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


Eigen::Matrix3f Position::get_rot_at(double ts){
////
// qt = (qc * qp^-1)^t * qp
// t = (ts - tp) / (tc - tp)

    QUAT::Quaternion quat_ts(quat_current);
    QUAT::Quaternion quat_temp(quat_past);

    quat_temp.invers();
    quat_ts.multiply(&quat_temp);
    quat_ts.pow((ts - t_past) / dt);
    quat_ts.multiply(&quat_past);

    //std::cout<<"current\n"<<quat_current.r<<std::endl;
    //std::cout<<"past\n"<<quat_past.r<<std::endl;
    //std::cout<<"interpolated\n"<<quat_ts.r<<std::endl;
    return quat_ts.r;
    
}

Eigen::Vector3f Position::get_pos_at(double ts){
    double t = (ts - t_past) / dt;
    double ti = 1 - t;

    //ROS_INFO("times t_current dt, ts, t, ti: %f %f %f %f %f", t_current, dt, ts, t, ti);
    Eigen::Vector3f pos_ts;
    pos_ts << ti * pos_past[0] + t * pos_current[0],
              ti * pos_past[1] + t * pos_current[1],
              ti * pos_past[2] + t * pos_current[2];

    //std::cout<<"pos at\n"<<pos_ts<<std::endl;
    //ROS_INFO("current %f %f %f", pos_current[0], pos_current[1], pos_current[2]);
    //ROS_INFO("past %f %f %f", pos_past[0], pos_past[1], pos_past[2]);
    //ROS_INFO("position %f %f %f", pos_ts[0], pos_ts[1], pos_ts[2]);
    return pos_ts;
}

Eigen::Vector3f Position::get_current_pos(){
    
    Eigen::Vector3f position;
    position << pos_current[0], pos_current[1], pos_current[2];
    //std::cout<<"pos pos\n"<<position<<std::endl;
    return position;
}

Eigen::Matrix3f Position::get_rot(){
    //std::cout<<"r\n"<<quat.r<<std::endl;
    //ROS_INFO("retrun rot");
    return quat_current.r;
}

void Position::dir_vector(double *dir){
    quat_current.rotate(dir);
}


} //namespace
