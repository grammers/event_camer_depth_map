#include <voxel_grid.hpp>

namespace GRID{

Voxel::Voxel(int dimX, int dimY, int dimZ){
    this->dimX = dimX;
    this->dimY = dimY;
    this->dimZ = dimZ;

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
    /*
    //ROS_INFO("arrived");
    double ref_pos [6];
    //ROS_INFO("test trace");
    ref_pos[0] = init_pos[0] + cam_pos[0];
    ref_pos[1] = init_pos[1] + cam_pos[1];
    ref_pos[2] = init_pos[2] + cam_pos[2];
    ref_pos[3] = init_pos[3] + cam_pos[3];
    //ROS_INFO("pre atan");
    ref_pos[4] = init_pos[4] + cam_pos[4] + atan(event_dir[1] / event_dir[0]);
    ref_pos[5] = init_pos[5] + cam_pos[5] + acos(event_dir[2] / sqrt(pow(event_dir[0],2) + pow(event_dir[1],2) + pow(event_dir[2],2)));

    //ROS_INFO("post ref_pos");

    double px = cos(ref_pos[5]) * cos(ref_pos[4]);
    double py = sin(ref_pos[5]) * cos(ref_pos[4]);
    double pz = sin(ref_pos[4]);

    //ROS_INFO("ref %f %f %f", ref_pos[3], ref_pos[4], ref_pos[5]);
    //ROS_INFO("event dir %f %f %f", event_dir[0], event_dir[1], event_dir[2]);
    //ROS_INFO("px,py,pz %f %f %f", px, py, pz);

    int start_x = (int) (init_pos[0] + cam_pos[0] * 10);
    int start_y = (int) (init_pos[1] + cam_pos[1] * 10);
    int start_z = (int) (init_pos[2] + cam_pos[2] * 10);
    //ROS_INFO("cam_pos %f %f %f", cam_pos[0], cam_pos[1], cam_pos[2]);
    //ROS_INFO("start %i %i %i", start_x, start_y, start_z);
    //ROS_INFO("param set upp");
*/

    for(int i = 0; i < 3; i++){
        cam_pos[i] *= 10;
    }
    double ray [6];
    setup(cam_pos, event_dir, ray);

    int start_x = (int) (ray[0]);
    int start_y = (int) (ray[1]);
    int start_z = (int) (ray[2]);

    int change = ray_direction(ray[3]);
    //ROS_INFO("X %i", start_x + change);
    for (int X = start_x + change; X < dimX && X >= 0; X += change){
        //ROS_INFO("loop x %i", X);
        double t = (X - ray[0]) / ray[3];
        if (!add_hit(t, ray[0], ray[1], ray[2], ray[3], ray[4], ray[5])){
            break;
        }
    }

    change = ray_direction(ray[4]);
    for (int Y = start_y + change; Y < dimY && Y >= 0; Y += change){
        //ROS_INFO("loop y %i", Y);
        double t = (Y - ray[1]) / ray[4];
        if (!add_hit(t, ray[0], ray[1], ray[2], ray[3], ray[4], ray[5])){
            break;
        }
    }

    change = ray_direction(ray[5]);
    for (int Z = start_z + change; Z < dimZ && Z >= 0; Z += change){
        //ROS_INFO("loop z %i", Z);
        double t = (Z - ray[2]) / ray[5];
        if (!add_hit(t, ray[0], ray[1], ray[2], ray[3], ray[4], ray[5])){
            break;
        }
    }
   /* 
    ROS_INFO("read");
    for (int i = 0; i < grid.size(); i++){
        if (grid[i] != 0){
            ROS_INFO("voxel value %i", grid[i]);
        }
    }
    */
}

void Voxel::setup(double *camera_pos, double *event_dir_vector, double *ray){
    //ROS_INFO("in value %f %f", camera_pos[0], event_dir_vector[0]);
    ray[0] = camera_pos[0] + init_pos[0];
    ray[1] = camera_pos[1] + init_pos[1];
    ray[2] = camera_pos[2] + init_pos[2];

    double r0 = camera_pos[3] + init_pos[3];
    double p0 = camera_pos[4] + init_pos[4] + atan(event_dir_vector[1] / event_dir_vector[2]);
    //double j0 = camera_pos[5] + init_pos[5] + acos(event_dir_vector[2] / sqrt(pow(event_dir_vector[0],2) + pow(event_dir_vector[1],2) + pow(event_dir_vector[2],2)));
    double j0 = camera_pos[5] + init_pos[5] + atan(event_dir_vector[0] / event_dir_vector[2]);

    //ROS_INFO("pj %f %f", p0, j0);

    ray[3] = cos(j0) * cos(p0);
    ray[4] = sin(j0) * cos(p0);
    ray[5] = sin(p0);
}

int Voxel::ray_direction(double p){
    if (p > 0) {
        return 1;
    }
    return -1;
}

bool Voxel::add_hit(double t, int start_x, int start_y, int start_z,
                        double px, double py, double pz){

    int x = (int) (start_x + (px * t));
    int y = (int) (start_y + (py * t));
    int z = (int) (start_z + (pz * t));

    //ROS_INFO("xyz set");

    if (!in_bound(x, y, z)){
        //ROS_INFO("out of bound %i %i %i", x, y, z);
        return false;
    }
    
    //ROS_INFO("time to add %i %i %i", x, y, z);
    grid[x + dimX * (y + dimY * z)]++;
    //ROS_INFO("++ %i", grid[x + dimX * (y + dimY *z)]);

    return true;
}

void Voxel::hit_id(double t, double *ray, int *index){
    for (int i = 0; i < 3; i++){
        index[i] = (int) (ray[i] + t * ray[i + 3]);
    }
}

bool Voxel::in_bound(int x, int y, int z){
    if (x >= dimX || x < 0 ||
        y >= dimY || y < 0 ||
        z >= dimZ || z < 0){

        return false;
    }
    return true;
                
}

double Voxel::distans(int *index){
   // ROS_INFO("%i", grid[index[0] + dimX * (index[1] + dimY * index[2])]);
    if (grid[index[0] + dimX * (index[1] + dimY * index[2])] > 3){
        //ROS_INFO("%f", sqrt(pow(index[0],2) + pow(index[1],2) + pow(index[2],2)));
        return sqrt(pow(index[0],2) + pow(index[1],2) + pow(index[2],2));
    }
    return 0.0;
}

double Voxel::depth_at_pixel(double *cam_pos, double *pixel_vector){
    for(int i = 0; i < 3; i++){
        cam_pos[i] *= 10;
    }
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

    int start_x = (int) (ray[0]);
    int start_y = (int) (ray[1]);
    int start_z = (int) (ray[2]);
    
    //ROS_INFO("start %i %i %i", start_x, start_y, start_z);

    double min_dist = dimX;

    int index [3];
    int change = ray_direction(ray[3]);
    //ROS_INFO("wtf %i", start_x + change);
    //ROS_INFO("delta %f", ray[3]);
    for(int X = start_x + change; X < dimX && X >= 0; X += change){
        if (ray[3] == 0){
            break;
        }
        double t = (X - ray[0]) / ray[3];
        hit_id(t, ray, index);
        ROS_INFO("hit id X %i %i %i", index[0], index[1], index[2]);
        if(!in_bound(index[0], index[1], index[2])){
            break;
        }
        //ROS_INFO("time for distans");
        double distans_x = distans(index);
        if (distans_x != 0.0){
            if (distans_x < min_dist){
                min_dist = distans_x;
            }
            break;
        }
        
    }

    change = ray_direction(ray[4]);
    for(int Y = start_y + change; Y < dimY && Y >= 0; Y += change){
        if (ray[4] == 0){
            break;
        }
        double t = (Y - ray[1]) / ray[4];
        hit_id(t, ray, index);
        ROS_INFO("hit id Y %i %i %i", index[0], index[1], index[2]);
        if(!in_bound(index[0], index[1], index[2])){
            break;
        }
        double distans_x = distans(index);
        if (distans_x != 0){
            if (distans_x < min_dist){
                min_dist = distans_x;
            }
            break;
        }
        
    }

    change = ray_direction(ray[5]);
    for(int Z = start_z + change; Z < dimZ && Z >= 0; Z += change){
        if (ray[5] == 0){
            break;
        }
        double t = (Z - ray[3]) / ray[5];
        hit_id(t, ray, index);
        ROS_INFO("hit id Z %i %i %i", index[0], index[1], index[2]);
        if(!in_bound(index[0], index[1], index[2])){
            break;
        }
        double distans_x = distans(index);
        if (distans_x != 0){
            if (distans_x < min_dist){
                min_dist = distans_x;
            }
            break;
        }
        
    }
    return min_dist;
}


} //namespace
