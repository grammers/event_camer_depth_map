#include <voxel_grid.hpp>

#define THRES 0.1 //10000 //4500 //10000
//#define THRES 1000 //4500 //10000
#define MIN_DIST 10

namespace GRID{

Voxel::Voxel(ODOM::Position *p, int dimX, int dimY, int dimZ){
    this->pos = p;

    dim[0] = dimX;
    dim[1] = dimY;
    dim[2] = dimZ;

    init_pos[0] = 10;
    init_pos[1] = dimY / 2;
    init_pos[2] = dimZ / 2;
    
    grid.resize(dimX * dimY * dimZ);

    std::fill(grid.begin(), grid.end(), 0);
}


void Voxel::add_ray(double *event_dir){
    //ROS_INFO("event %f %f %f", event_dir[0], event_dir[1], event_dir[2]);
    pos->dir_vector(event_dir);
    //ROS_INFO("dir %f %f %f", event_dir[0], event_dir[1], event_dir[2]);
    double *cam_pos = pos->get_current_pos();
    double ray [6];

    setup(cam_pos, event_dir, ray);

    for (int i = 0; i < 3; i++){
        int change = ray_direction(ray[i + 3]);
        for (int plain = (int)ray[i] + change * MIN_DIST; plain < dim[i] && plain >= 0; plain += change){
            double t = (plain - ray[i]) / ray[i + 3];
            if (!add_hit(t, ray)){
                break;
            }
        }
    }
}

void Voxel::setup(double *camera_pos, double *event_dir_vector, double *ray){
    ray[0] = camera_pos[0] * RESULUTION + init_pos[0];
    ray[1] = camera_pos[1] * RESULUTION + init_pos[1];
    ray[2] = camera_pos[2] * RESULUTION + init_pos[2];
    
    ray[3] = event_dir_vector[0]; 
    ray[4] = event_dir_vector[1];
    ray[5] = event_dir_vector[2];

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
        return false;
    }
    
    grid[index[0] + dim[0] * (index[1] + dim[1] * index[2])]++;

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

int Voxel::nr_ray(int x, int y, int z){
    return grid[x + dim[0] * (y + dim[1] * z)];
}



double Voxel::distans(int *index, double *ray){
    if (grid[index[0] + dim[0] * (index[1] + dim[1] * index[2])] > THRES){
        return sqrt(pow(index[0] - ray[0],2) + pow(index[1] - ray[1],2) + pow(index[2] - ray[2],2));
    }
    return 0.0;
}

void Voxel::clear(){
    int size = dim[0] * dim[1] * dim[2];
    for (int i = 0; i < size; i++){
        grid[i] = 0;
    }
}

void Voxel::max_nr_ray(double* direction, int w, int h){
    pos->dir_vector(direction);
    double *cam_pos = pos->get_current_pos();
    double ray [6];
    setup(cam_pos, direction, ray);
    int this_max = 0;
    int max_index[3] {dim[0], dim[1], dim[2]};

    for (int i = 0; i < 3; i++){
        int change = ray_direction(ray[i + 3]);
        for (int plain = (int)ray[i] + change; plain < dim[i] && plain >= 0; plain += change){
            double t = (plain - ray[i]) / ray[i + 3];
            int index [3];
            hit_id(t, ray, index);
            if(!in_bound(index)){
                break;
            }
            int nr = nr_ray(index[0], index[1], index[2]);
            if (nr > this_max){
                this_max = nr;
                max_index[0] = index[0];
                max_index[1] = index[1];
                max_index[2] = index[2];
            }

        }
    }
    float depth = ((float)(sqrt(
        pow(max_index[0] - ray[0], 2) + 
        pow(max_index[1] - ray[1], 2) + 
        pow(max_index[2] - ray[2], 2)) / MAX_DEPTH) * 255);

    max_dist.at<uchar>(h,w) = (uchar) depth;

    //ROS_INFO("%f",depth);
    if (depth < 175){
        nr++;
        med += ((depth / 255) * MAX_DEPTH) / 33;
    }
}

void Voxel::filter(int width, int height, double fx, double fy){
    // for tuning
    nr = 0;
    med = 0;

    max_dist = cv::Mat(height, width, CV_8U, cv::Scalar(255));
    // colaps z 
    for (int w = 0; w < width; w++){
        for (int h = 0; h < height; h++){
            double direction[3] = {
            1, 
            -(w - width/2) / fx, 
            -(h - height/2) / fy
            };

            max_nr_ray(direction, w, h);
        }
    }
    //ROS_INFO("med %f %i", med / (float) nr, nr);

}

int Voxel::filtered_mark(int w, int h){
    return (int) max_dist.at<uchar>(h,w);
}

void Voxel::depth_map(cv::Mat& img){
    img = max_dist.clone();
}

} //namespace
