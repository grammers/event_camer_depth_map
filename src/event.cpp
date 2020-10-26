#include <event.hpp>

namespace EVENT{

Event::Event(ODOM::Position *p, GRID::Voxel *grid) {
    pos = p;
    this->grid = grid;
}

void Event::event_callback(const dvs_msgs::EventArray::ConstPtr& msg){
    ROS_INFO("event");
    int size = msg->events.size();
    for (int i = 0;  i < size; i++){
        const dvs_msgs::Event& e = msg->events[i];
        double* event_pos = pos->pos_at(e.ts.toSec());
        double event_pos_camera [3];

        event_pos_camera[1] = -e.x + cx; // widht
        event_pos_camera[2] = -e.y + cy; // height
        event_pos_camera[0] = f; // depth

        //ROS_INFO("event %f %f %f", event_pos[0], event_pos[1], event_pos[2]);
        grid->add_ray(event_pos, event_pos_camera);

    }
}

double Event::get_f(){
    return f;
}

void Event::set_camera(double fx, double fy, double cx, double cy){
    this->f = (fx + fy) / 2;
    this->cx = cx;
    this->cy = cy;
    //ROS_INFO("cx cy, %f %f", cx, cy);
    return;
}

} // namespace
