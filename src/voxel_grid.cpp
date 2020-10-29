#include <voxel_grid.hpp>

#define THRES 0.9 //10000 //4500 //10000
#define MIN_DIST 16

namespace GRID{

Voxel::Voxel(int dimX, int dimY, int dimZ){
    //this->dimX = dimX;
    //this->dimY = dimY;
    //this->dimZ = dimZ;

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

    //ROS_INFO("setup");
    max = cv::Mat(dimX,dimY, CV_32FC1, cv::Scalar(0));
    max_coordinates = cv::Mat(dimX, dimY, CV_32FC1, cv::Scalar(0));
    max_filtered_coordinates = cv::Mat(dimX, dimY, CV_32FC1, cv::Scalar(0));
    //ROS_INFO("complet");
    //mask = cv::Mat(dimX, dimY, CV_U8);

}


void Voxel::add_ray(double *cam_pos, double *event_dir){

    double ray [6];
    setup(cam_pos, event_dir, ray);

    for (int i = 0; i < 3; i++){
        int change = ray_direction(ray[i + 3]);
        //ROS_INFO("X %f %i %i", ray[i], change,  MIN_DIST);
        for (int plain = (int)ray[i] + change * MIN_DIST; plain < dim[i] && plain >= 0; plain += change){
            //ROS_INFO("loop x %i", plain);
            double t = (plain - ray[i]) / ray[i + 3];
            if (!add_hit(t, ray)){
                break;
            }
        }
    }
}

void Voxel::setup(double *camera_pos, double *event_dir_vector, double *ray){
    //ROS_INFO("in value %f %f", camera_pos[0], init_pos[0]);
    double i = camera_pos[3];
    double j = camera_pos[4];
    double k = camera_pos[5];
    double r = camera_pos[6];


    ray[0] = camera_pos[0] * resulution + init_pos[0];
    ray[1] = camera_pos[1] * resulution + init_pos[1];
    ray[2] = camera_pos[2] * resulution + init_pos[2];
    
    // quart rot
    ray[3] = event_dir_vector[0] * (1 - 2 * pow(j,2) - 2 * pow(k,2))
             + event_dir_vector[1] * 2 * (i * j - k * r)
             + event_dir_vector[2] * 2 * (i * k + j *r);
    ray[4] = event_dir_vector[0] * 2 * (i * j + k * r)
             + event_dir_vector[1] * (1 - 2 * pow(i,2) - 2 * pow(k,2))
             + event_dir_vector[2] * 2 * (j * k - i * r);
    ray[5] = event_dir_vector[0] * 2 * (i * k - j * r)
             + event_dir_vector[1] * 2 * (i * r + j * k)
             + event_dir_vector[2] * (1 - 2 * pow(i,2) - 2 * pow(j,2));

/*
    //euiler rot
    ray[3] = event_dir_vector[0] * cos(p) * cos(y) 
            - event_dir_vector[1] * cos(p) * sin(y) 
            + event_dir_vector[2] * cos(p);
    ray[4] = event_dir_vector[0] * (sin(r) * sin(p) * cos(y) + cos(r) * sin(y)) 
            - event_dir_vector[1] * (sin(r) * sin(p) * sin(y) + cos(r) * cos(y)) 
            - event_dir_vector[2] * sin(r) * sin(p);
    ray[5] = event_dir_vector[0] * (cos(r) * sin(p) * cos(y) + sin(r) * cos(y))
            + event_dir_vector[1] * (cos(r) * sin(p) * sin(y) + sin(r) * cos(y))
            + event_dir_vector[2] * cos(r) * cos(p);

*/
/*
    double r0 = camera_pos[3] + init_pos[3];
    double p0 = camera_pos[4] + init_pos[4] + atan(event_dir_vector[1] / event_dir_vector[2]);
   // double j0 = camera_pos[5] + init_pos[5] + acos(event_dir_vector[2] / sqrt(pow(event_dir_vector[0],2) + pow(event_dir_vector[1],2) + pow(event_dir_vector[2],2)));
    double j0 = camera_pos[5] + init_pos[5] + atan(event_dir_vector[0] / event_dir_vector[2]);

    //ROS_INFO("pj %f %f", p0, j0);

    ray[3] = cos(j0) * cos(p0);
    ray[4] = sin(j0) * cos(r0);
    ray[5] = sin(p0);
*/
    //ROS_INFO("changes %f %f %f", ray[3], ray[4], ray[5]);
    //ROS_INFO("pos %f %f %f", ray[0], ray[1], ray[2]);
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
        //ROS_INFO("out of bound %i %i %i", index[0], index[1], index[2]);
        return false;
    }
    
    //ROS_INFO("time to add %i %i %i", x, y, z);
    grid[index[0] + dim[0] * (index[1] + dim[1] * index[2])]++;
    //ROS_INFO("++ %i", grid[index[0] + dim[0] * (index[1] + dim[1] * index[2])]);

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

void Voxel::normalise(){
    max_ray = 0;
    int size = grid.size();
    for (int i = 0; i < size; i++){
        if (grid[i] > max_ray) { max_ray = grid[i];}
    }
}

int Voxel::nr_ray(int x, int y, int z){
    return grid[x + dim[0] * (y + dim[1] * z)];
}


bool Voxel::is_marked(int x, int y, int z){
    //ROS_INFO("where %i",grid[x + dim[0] * (y + dim[1] * z)]);
    if (((double) grid[x + dim[0] * (y + dim[1] * z)] / (double) max_ray) > THRES){
        return true;
    }
    return false;
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
    return min_dist;
}

void Voxel::filter(){
    // colaps z 
    int tot_max = 0;
    for(int z = 0; z < dim[2]; z++){
        for (int y = 0; y < dim[1]; y++){
            for (int x = 0; x < dim[0]; x++){
                if (nr_ray(x,y,z) > max.at<float>(z,y)){
                    max.at<float>(z,y) = (float) nr_ray(x,y,z);

                    max_coordinates.at<float>(z,y) = (float) x;
                    if (nr_ray(x,y,z) > tot_max){
                        tot_max = nr_ray(x,y,z);
                    }
                }
            }
        }
    }
    //ROS_INFO("colapsed");

    /*
    //normalise
    for(int x = 0; x < dim[0]; x++){
        for(int y = 0; y < dim[1]; y++){
            max.at<float>(x,y) = (max.at<float>(x,y) / (float)tot_max) * 255.0;
       }
    }
    */
    cv::Mat max_8;
    cv::normalize(max, max_8, 0.0, 255.0, cv::NORM_MINMAX);
    max_8.convertTo(max_8, CV_8U);
    
    //ROS_INFO("normlised");

    //addaptiv
    cv::adaptiveThreshold(max_8, mask, 1, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 5, 1.);

    //ROS_INFO("addapted");
    // filter
    for(int z = 0; z < dim[2]; z++){
        for(int y = 0; y < dim[1]; y++){
            if(mask.at<float>(z,y) > 0){
                max_filtered_coordinates.at<float>(z,y) = max_coordinates.at<float>(z,y);
            }
        }
    }
}

int Voxel::filtered_mark(int z, int y){
    //ROS_INFO("retreve");
    return (int) max_filtered_coordinates.at<float>(z,y);
}

} //namespace
