#include <event.hpp>

namespace EVENT{

Event::Event(GRID::Voxel *grid) {
    this->grid = grid;
}

void Event::event_callback(const prophesee_event_msgs::EventArray::ConstPtr& msg){
    int size = msg->events.size();
    for (int i = 0;  i < size; i++){
        const prophesee_event_msgs::Event& e = msg->events[i];
        double event_pos_camera [3];

        event_pos_camera[0] = 1.0; // depth
        event_pos_camera[1] = (e.y - cy) / fy; // height
        event_pos_camera[2] = -(e.x - cx) / fx; // widht
    
        grid->add_ray(event_pos_camera);

    }
}

double Event::get_f(){
    return f;
}

double Event::get_fx(){
    return fx;
}

double Event::get_fy(){
    return fy;
}

void Event::set_camera(double fx, double fy, double cx, double cy){
    this->f = (fx + fy) / 2;
    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;
    return;
}

} // namespace
