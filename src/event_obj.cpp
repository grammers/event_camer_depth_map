#include <event_obj.hpp>

namespace EVENTOBJ{

EventObj::EventObj(dvs_msgs::Event e, ODOM::Position *pos) {
//EventObj::EventObj(prophesee_event_msgs::Event e, ODOM::Position *pos) {
    event = e;
    v_pos = pos->get_pos_at(get_ts());
    r_pos = pos->get_rot_at(get_ts());
    //v_pos = pos->get_current_pos();
    //r_pos = pos->get_rot();
    //ODOM::Position cont_pos(pos);
    //this->pos = &cont_pos;
    //Eigen::Vector3f test = this->pos->get_current_pos();
    //std::cout<<"event obj\n"<<e<<std::endl;
    //std::cout<<"pos event test\n"<<pos->get_current_pos()<<std::endl;
}

float EventObj::get_x(){
    //std::cout<<"x: "<<event.x<<std::endl;
    return (float) event.x;
}
float EventObj::get_y(){
    return (float) event.y;
}

double EventObj::get_ts(){
    return event.ts.toSec();
}

Eigen::Vector3f EventObj::get_pos(){
    //Eigen::Vector3f test = this->pos->get_current_pos();
    //std::cout<<"pos event obj\n"<<v_pos<<std::endl;
    return v_pos;
    ROS_INFO("GET POS OBJ");
    //return *pos;
}

Eigen::Matrix3f EventObj::get_rot(){
    //std::cout<<"rot event obj\n"<<r_pos<<std::endl;
    return r_pos; //pos->get_rot();
}

} //namespace
