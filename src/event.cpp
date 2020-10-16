#include <event.hpp>

namespace EVENT{

Event::Event(ODOM::Position p, GRID::Voxel *grid) {
    pos = p;
    this->grid = grid;
}

void Event::event_callback(const dvs_msgs::EventArray::ConstPtr& msg){
    ROS_INFO("event");
    int size = msg->events.size();
    for (int i = 0;  i < size; i++){
        const dvs_msgs::Event& e = msg->events[i];
        double* event_pos = pos.pos_at(e.ts.toSec());
        double event_pos_camera [3];

        event_pos_camera[0] = e.x - cx;
        event_pos_camera[1] = e.y - cy;
        event_pos_camera[2] = f;

    }
}

void Event::set_camera(double fx, double fy, double cx, double cy){
    this->f = (fx + fy) / 2;
    this->cx = cx;
    this->cy = cy;
    return;
}

} // namespace
