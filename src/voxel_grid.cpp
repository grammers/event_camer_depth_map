#include <voxel_grid.hpp>

#define THRES 200
#define MIN_DIST 30

namespace GRID{

Voxel::Voxel(int dimX, int dimY, int dimZ){
    this->dimX = dimX;
    this->dimY = dimY;
    this->dimZ = dimZ;

    dim[0] = dimX;
    dim[1] = dimY;
    dim[2] = dimZ;

    init_pos[0] = dimX / 2;
    init_pos[1] = dimY / 2;
    init_pos[2] = dimZ / 2;
    init_pos[3] = 0;
    init_pos[4] = 0;
    init_pos[5] = 0;
    
    grid.resize(dimX * dimY * dimZ);

    std::fill(grid.begin(), grid.end(), 0);

}


void Voxel::add_ray(double *cam_pos, double *event_dir){

    double ray [6];
    setup(cam_pos, event_dir, ray);

    for (int i = 0; i < 3; i++){
        int change = ray_direction(ray[i + 3]);
        //ROS_INFO("X %i", start_x + change);
        for (int plain = (int)ray[i] + change * MIN_DIST; plain < dim[i] && plain >= 0; plain += change){
            //ROS_INFO("loop x %i", X);
            double t = (plain - ray[i]) / ray[i + 3];
            if (!add_hit(t, ray)){
                break;
            }
        }
    }
}

void Voxel::setup(double *camera_pos, double *event_dir_vector, double *ray){
    //ROS_INFO("in value %f %f", camera_pos[0], event_dir_vector[0]);
    ray[0] = camera_pos[0] * resulution + init_pos[0];
    ray[1] = camera_pos[1] * resulution + init_pos[1];
    ray[2] = camera_pos[2] * resulution + init_pos[2];

    double r0 = camera_pos[3] + init_pos[3];
    double p0 = camera_pos[4] + init_pos[4] + atan(event_dir_vector[1] / event_dir_vector[2]);
   // double j0 = camera_pos[5] + init_pos[5] + acos(event_dir_vector[2] / sqrt(pow(event_dir_vector[0],2) + pow(event_dir_vector[1],2) + pow(event_dir_vector[2],2)));
    double j0 = camera_pos[5] + init_pos[5] + atan(event_dir_vector[0] / event_dir_vector[2]);

    //ROS_INFO("pj %f %f", p0, j0);

    ray[3] = cos(j0) * cos(p0);
    ray[4] = sin(j0) * cos(r0);
    ray[5] = sin(p0);
    //ROS_INFO("changes %f %f %f", ray[3], ray[4], ray[5]);
}

int Voxel::ray_direction(double p){
    if (p > 0) {
        return 1;
    }
    return -1;
}

bool Voxel::add_hit(double t, double *ray){
    int index [3];
    hit_id(t, ray, index);

    if (!in_bound(index)){
        //ROS_INFO("out of bound %i %i %i", x, y, z);
        return false;
    }
    
    //ROS_INFO("time to add %i %i %i", x, y, z);
    grid[index[0] + dim[0] * (index[1] + dim[1] * index[2])]++;
    //ROS_INFO("++ %i", grid[x + dimX * (y + dimY *z)]);

    return true;
}

void Voxel::hit_id(double t, double *ray, int *index){
    for (int i = 0; i < 3; i++){
        index[i] = (int) (ray[i] + t * ray[i + 3]);
    }
}

bool Voxel::in_bound(int *index){
    for (int i = 0; i < 3; i++){
        if (index[i] >= dim[i] || index[i] < 0){
            return false;
        }
    }
    return true;
                
}

double Voxel::distans(int *index, double *ray){
   // ROS_INFO("%i", grid[index[0] + dimX * (index[1] + dimY * index[2])]);
    if (grid[index[0] + dim[0] * (index[1] + dim[1] * index[2])] > THRES){
        //ROS_INFO("%f", sqrt(pow(index[0],2) + pow(index[1],2) + pow(index[2],2)));
        return sqrt(pow(index[0] - ray[0],2) + pow(index[1] - ray[1],2) + pow(index[2] - ray[2],2));
    }
    return 0.0;
}

double Voxel::depth_at_pixel(double *cam_pos, double *pixel_vector){
    //ROS_INFO("norm pos %f", cam_pos[0]);
    /* 
    //ROS_INFO("read");
    for (int i = 0; i < grid.size(); i++){
        if (grid[i] != 0){
            ROS_INFO("voxel value %i", grid[i]);
        }
    }
    */
    double ray [6];
    setup(cam_pos, pixel_vector, ray);

    
    //ROS_INFO("start %i %i %i", start_x, start_y, start_z);

    double min_dist = dimX;
    for (int i = 0; i < 3; i++){
        int index [3];
        int change = ray_direction(ray[i + 3]); 
        //ROS_INFO("wtf %i", start_x + change); 
        //ROS_INFO("delta %f %f %f", ray[3], ray[4], ray[5]);
        for(int plain = (int)ray[i] + change * 10; plain < dim[i] && plain >= 0; plain += change){
            if (ray[i + 3] == 0){
                //ROS_INFO("is 0");
                break;
            }
            double t = (plain - ray[i]) / ray[i + 3];
            hit_id(t, ray, index);
            //ROS_INFO("hit id X %i %i %i", index[0], index[1], index[2]);
            if(!in_bound(index)){
                break;
            }
            //ROS_INFO("time for distans");
            double dist = distans(index, ray);
            if (dist != 0.0){
                if (dist < min_dist){
                    min_dist = dist;
                }
                break;
            }
            
        }

    }
/*
    int index [3];
    int change = ray_direction(ray[3]);
    //ROS_INFO("wtf %i", start_x + change);
    //ROS_INFO("delta %f %f %f", ray[3], ray[4], ray[5]);
    for(int X = start_x + change * 10; X < dimX && X >= 0; X += change){
        if (ray[3] == 0){
            //ROS_INFO("is 0");
            break;
        }
        double t = (X - ray[0]) / ray[3];
        hit_id(t, ray, index);
        //ROS_INFO("hit id X %i %i %i", index[0], index[1], index[2]);
        if(!in_bound(index)){
            break;
        }
        //ROS_INFO("time for distans");
        double distans_x = distans(index, ray);
        if (distans_x != 0.0){
            if (distans_x < min_dist){
                min_dist = distans_x;
            }
            break;
        }
        
    }

    change = ray_direction(ray[4]);
    for(int Y = start_y + change * 10; Y < dimY && Y >= 0; Y += change){
        if (ray[4] == 0){
            break;
        }
        double t = (Y - ray[1]) / ray[4];
        hit_id(t, ray, index);
        //ROS_INFO("hit id Y %i %i %i", index[0], index[1], index[2]);
        if(!in_bound(index)){
            break;
        }
        //ROS_INFO("t f dist y");
        double distans_x = distans(index, ray);
        if (distans_x != 0.0){
            if (distans_x < min_dist){
                min_dist = distans_x;
            }
            break;
        }
        
    }

    change = ray_direction(ray[5]);
    for(int Z = start_z + change * 10; Z < dimZ && Z >= 0; Z += change){
        if (ray[5] == 0){
            break;
        }
        double t = (Z - ray[2]) / ray[5];
        hit_id(t, ray, index);
        //ROS_INFO("hit id Z %i %i %i", index[0], index[1], index[2]);
        if(!in_bound(index)){
            break;
        }
        //ROS_INFO("t f dist Z");
        double distans_x = distans(index, ray);
        if (distans_x != 0.0){
            if (distans_x < min_dist){
                min_dist = distans_x;
            }
            break;
        }
        
    }
*/
    return min_dist;
}


} //namespace
