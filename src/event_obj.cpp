#include <event_obj.hpp>

namespace EVENTOBJ{

EventObj::EventObj(dvs_msgs::Event e) {
    event.ts = e.ts;
    event.x = e.x;
    event.y = e.y;
    event.polarity = e.y;
}

EventObj::EventObj(prophesee_event_msgs::Event e) {
    event = e;
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
ros::Time EventObj::get_ros_ts(){
    return event.ts;
}

} //namespace
