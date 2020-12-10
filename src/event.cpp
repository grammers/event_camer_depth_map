#include <event.hpp>

namespace EVENT{

Event::Event(GRID::Voxel *grid) {
    this->grid = grid;
}

void Event::event_dvs_callback(const dvs_msgs::EventArray::ConstPtr& msg){
    int size = msg->events.size();
    for (int i = 0;  i < size; i++){
        const dvs_msgs::Event& e = msg->events[i];
        EVENTOBJ::EventObj eo(e);
        event_obj.push_back(eo);
    }
}        
void Event::event_pro_callback(const prophesee_event_msgs::EventArray::ConstPtr& msg){
    int size = msg->events.size();
    for (int i = 0;  i < size; i++){
        const prophesee_event_msgs::Event& e = msg->events[i];
        EVENTOBJ::EventObj eo(e);
        event_obj.push_back(eo);
    }
}


void Event::add_ray(LinearTrajectory *trajectory, geometry_utils::Transformation *T_cw){
    int stop = event_obj.size();
    ROS_INFO("nr events %i", stop);
    for(int i = 0; i < stop; i++){
        grid->add_ray_traj(&event_obj.at(i), trajectory, T_cw);
    }
    event_obj.clear();
}

} // namespace
