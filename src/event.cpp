#include <event.hpp>

namespace EVENT{

Event::Event(GRID::Voxel *grid, ODOM::Position *pos) {
    this->pos = pos;
    this->grid = grid;
}

void Event::event_callback(const prophesee_event_msgs::EventArray::ConstPtr& msg){
    int size = msg->events.size();
    for (int i = 0;  i < size; i++){
        const prophesee_event_msgs::Event& e = msg->events[i];
        //ODOM::Position e_pos(*pos);
        EVENTOBJ::EventObj eo(e, pos);
        event_obj.push_back(eo);
        /*
        if(event_obj.begin() == event_obj.end()){
            event_obj.insert(event_obj.begin(), eo);
        }
        else if(eo.get_ts() > event_obj.end()->get_ts()){
            event_obj.push_back(eo);
        }
        else{
            //std::cout<<"add"<<std::endl;
            for (int i = 0; i < event_obj.size(); i++){
                if(event_obj.at(i).get_ts() <= eo.get_ts()){
                    event_obj.insert(event_obj.end() - i, eo);
                    break;
                }

            }
        }
        */
        /*
        double event_pos_camera [3];

        event_pos_camera[0] = 1.0; // depth
        event_pos_camera[1] = -(e.x - cx) / fx; // widht
        event_pos_camera[2] = -(e.y - cy) / fy; // height
    
        grid->add_ray(event_pos_camera);
        */

    }
}

void Event::dropout(){
    int deadlin_ts = event_obj.back().get_ts() - TS;
    std::cout<<deadlin_ts<<" > "<<event_obj.begin()->get_ts()<<std::endl;
    while(deadlin_ts > event_obj.begin()->get_ts()){
        event_obj.erase(event_obj.begin());
    }
}

void Event::add_ray(){
    //std::for_each(event_obj.begin(), event_obj.end(), [=](EVENTOBJ::EventObj e) {grid->add_ray(e);});
    //dropout();
    int stop = event_obj.size();
    ROS_INFO("nr events %i", stop);
    for(int i = 0; i < stop; i++){
        //EVENTOBJ::EventObj e = event_obj.at(i);
        //std::cout<<"event add ray "<<event_obj.at(i).get_x()<<std::endl;
        grid->add_ray(&event_obj.at(i));
    }
    event_obj.clear();
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
