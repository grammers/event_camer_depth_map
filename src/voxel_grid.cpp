#include <voxel_grid.hpp>

#define THRES 0.1 //10000 //4500 //10000
//#define THRES 1000 //4500 //10000
#define MIN_DIST 10

namespace GRID{

Voxel::Voxel(ODOM::Position *p, int dimX, int dimY, int dimZ, int FX, int FY, int CX, int CY){
    this->pos = p;
    //this->FX = FX;
    //this->FY = FY;
    //this->CX = CX;
    //this->CY = CY;
    
    this->FX = FX;
    this->FY = FY;
    this->CX = CX;
    this->CY = CY;
    

    K << FX, 0, CX,
        0, FY, CY,
        0, 0, 1;


    dim[0] = dimX;
    dim[1] = dimY;
    dim[2] = dimZ;

    init_pos[0] = 10;
    init_pos[1] = dimY / 2;
    init_pos[2] = dimZ / 2;
    
    grid.resize(dimX * dimY * dimZ);

    std::fill(grid.begin(), grid.end(), 0);

    max_dist = cv::Mat(dimY, dimX, CV_8U, cv::Scalar(255));
}

void Voxel::camInit(const sensor_msgs::CameraInfo *msg){
    dvs_cam.fromCameraInfo(*msg);
    K << dvs_cam.fx(), 0.f, dvs_cam.cx(),
         0.f, dvs_cam.fy(), dvs_cam.cy(),
         0.f, 0.f, 1.f;

    FX = dvs_cam.fx();
    FY = dvs_cam.fy();
    CX = dvs_cam.cx();
    CY = dvs_cam.cy();
}

void Voxel::add_ray_traj(EVENTOBJ::EventObj *e, LinearTrajectory *trajectory, geometry_utils::Transformation *T_cw){
    
    geometry_utils::Transformation T_we;
    geometry_utils::Transformation T_ce;
    
    if(!trajectory->getPoseAt(e->get_ros_ts(), T_we))
        return;

    T_ce = *T_cw * T_we;
    const geometry_utils::Transformation T_ec = T_ce.inverse();
    const Eigen::Matrix3f R = T_ec.getRotationMatrix().cast<float>();
    const Eigen::Vector3f t = T_ec.getPosition().cast<float>();
    
    const Eigen::Vector3f v_c = -R.transpose() * t; //camera center
    
    
    Eigen::Matrix3f h_z0= R;
    h_z0 *= Z0;
    h_z0.col(2) += t;

    h_z0 = K * h_z0 * K.inverse();
    h_z0 = h_z0.inverse().eval();

    Eigen::Matrix4f h_z0_4x4;
    h_z0_4x4.block<3,3>(0,0) = h_z0;
    h_z0_4x4.col(3).setZero();
    h_z0_4x4.row(3).setZero();


    Eigen::Vector4f p;
    cv::Point2d xy(e->get_x(), e->get_y());
    cv::Point2d rect_point = dvs_cam.rectifyPoint(xy);
    p << rect_point.x, rect_point.y, 1.0, 0.0;
    p = h_z0_4x4 * p;
    p /= p[2];

    for(int i = 1; i < dim[2]; i++){
        const float zi = Z0 + DEPTH * (float) i,
            a = Z0 * (zi - v_c[2]),
            bx = (Z0 - zi) * (v_c[0] * dvs_cam.fx()+ v_c[2] * dvs_cam.cx()),
            by = (Z0 - zi) * (v_c[1] * dvs_cam.fy() + v_c[2] * dvs_cam.cy()),
            d = zi * (Z0 - v_c[2]);

        int x = (int) ((p[0] * a + bx) / d);
        int y = (int) ((p[1] * a + by) / d);

        int index[3] {x,y,i};
        if(in_bound(index)){
            grid[x + dim[0] * (y + dim[1] * (dim[2] - i))] += 1;
        }
    }
}

void Voxel::add_ray(EVENTOBJ::EventObj *e){
    
    //ROS_INFO("add up");
    // naming convention v = pos vector, r =  rotation matrix, _xy y in x
    // w = world, c = camera/vue point, e = event pos
    Eigen::Vector3f v_wc = pos->get_current_pos();
    Eigen::Matrix3f r_wc = pos->get_rot();
    //std::cout<<"v_wc: " << v_wc.transpose()<<std::endl;
    //std::cout<<"r_wc: " << r_wc<<std::endl;
    
    // T ^ -1
    Eigen::Matrix4f t_cw;
    t_cw << r_wc, v_wc, 0.0,0.0,0.0,1.0;
    t_cw = t_cw.inverse().eval();
    // to world in camera
    //r_wc.transposeInPlace();
    //v_wc = r_wc * v_wc;

    Eigen::Vector3f v_we = e->get_pos();
    Eigen::Matrix3f r_we = e->get_rot();
    //std::cout<<"v_we^t\n"<<v_we.transpose()<<std::endl;
    //std::cout<<"r_we: " << r_we<<std::endl;

    // T1 * T2 
    Eigen::Matrix4f t_we;
    t_we << r_we, v_we, 0.0,0.0,0.0,1.0;
    //std::cout<<"r_wc\n"<<r_wc<<std::endl;
    //std::cout<<"r_we\n"<<r_we<<std::endl;
    //Eigen::Matrix3f r_ce = r_wc * r_we; // r_cw * r_we hear 
    //Eigen::Matrix4f T;
    //T << r_wc, v_wc, 0, 0, 0, 1;
    //Eigen::Vector4f V;
    //V << v_we, 1;
    //Eigen::Vector4f v_ce_4x = T * V;
    //Eigen::Vector3f v_ce = v_ce_4x.block<3,1>(0,0); // 

    Eigen::Matrix4f t_ce = t_cw * t_we;
    //std::cout<<"t_ce\n"<<t_ce<<std::endl;
    Eigen::Matrix3f r_ce = t_ce.block<3,3>(0,0);
    Eigen::Vector3f v_ce = t_ce.block<3,1>(0,3);
    //std::cout<<"v_ce\n"<<v_ce<<std::endl;

    // T ^ -1
    //std::cout<<"r_ce pre transp \n"<<r_ce<<std::endl;
    r_ce.transposeInPlace(); // r_ce egentligen r_ec (r)
    Eigen::Vector3f v_ec = r_ce * v_ce; // (t)

    Eigen::Vector3f v_c = -r_ce.transpose() * v_ec; // camera center
    //std::cout<<"v_ec: " << v_ec.transpose()<<std::endl;
    //std::cout<<"v_c: " << v_c.transpose()<<std::endl;

    
    Eigen::Matrix3f h_z0= r_ce;
    h_z0 *= Z0;
    //std::cout<<"h_z0_no v "<<h_z0<<std::endl;
    h_z0.col(2) += v_ec;

    //std::cout<<"h_z0_no K "<<h_z0<<std::endl;
    h_z0 = K * h_z0 * K.inverse();
    //std::cout<<"h_z0_inv "<<h_z0<<std::endl;
    h_z0 = h_z0.inverse().eval();
    //std::cout<<"h_z0 "<<h_z0<<std::endl;

    Eigen::Matrix4f h_z0_4x4;
    h_z0_4x4.block<3,3>(0,0) = h_z0;
    h_z0_4x4.col(3).setZero();
    h_z0_4x4.row(3).setZero();

    // ^^ myght be bosible to do erlier
    //h_z0_4x4 << 1,0,0,0,
    //            0,1,0,0,
    //            0,0,1,0,
    //            0,0,0,0;
    //v_c << 0,0,0;
    
    //Eigen::MatrixXf rect(3,4);
    //rect << 168.6294097900391, 0.0, 135.348079770296, 0.0,
            //0.0, 178.5641784667969, 113.6189973794753, 0.0,
            //0.0, 0.0, 1.0, 0.0;
    //Eigen::Vector3f p_pre;
    Eigen::Vector4f p;
    //p_pre << (float) e->get_x(), (float) e->get_y(), 1.0;
    //std::cout<<"p_pre\t"<<p_pre.transpose()<<std::endl;
    //Eigen::Vector3f p_rect = K.inverse() * p_pre;
    cv::Point2d xy(e->get_x(), e->get_y());
    //std::cout<<xy<<std::endl;
    cv::Point2d rect_point = dvs_cam.rectifyPoint(xy);
    //std::cout<<rect_point<<std::endl;
    //std::cout<<"p_rect\t"<<p_rect.transpose()<<std::endl;
    p << rect_point.x, rect_point.y, 1.0, 0.0;
    //std::cout<<"p\t"<<p.transpose()<<std::endl;
    p = h_z0_4x4 * p;
    p /= p[2];
    //std::cout<<"p after\n"<<p<<std::endl;
    // prep end //

    for(int i = 1; i < dim[2]; i++){
        const float zi = Z0 + DEPTH * (float) i,
            a = Z0 * (zi - v_c[2]),
            bx = (Z0 - zi) * (v_c[0] * dvs_cam.fx()+ v_c[2] * dvs_cam.cx()),
            by = (Z0 - zi) * (v_c[1] * dvs_cam.fy() + v_c[2] * dvs_cam.cy()),
            d = zi * (Z0 - v_c[2]);

        int x = (int) ((p[0] * a + bx) / d);
        int y = (int) ((p[1] * a + by) / d);

        //ROS_INFO("add %i %i %i", x,y,i);
        int index[3] {x,y,i};
        if(in_bound(index)){
            grid[x + dim[0] * (y + dim[1] * i)] += 1;
            //ROS_INFO("add %i %i %i", x,y,i);
            //ROS_INFO("calc %f %f %f %f", a,bx,by,d);
            //ROS_INFO("p %f %f", p[0], p[1]);
            //ROS_INFO("v_ce %f %f %f", v_ce[0], v_ce[1], v_ce[2]);
        }
    }

    //ROS_INFO("event %f %f %f", event_dir[0], event_dir[1], event_dir[2]);
    //pos->dir_vector(event_dir);
    //ROS_INFO("dir %f %f %f", event_dir[0], event_dir[1], event_dir[2]);
    //double *cam_pos = pos->get_current_pos();
    //double ray [6];

    //setup(cam_pos, event_dir, ray);
    /*
    for (int i = 0; i < 3; i++){
        int change = ray_direction(ray[i + 3]);
        for (int plain = (int)ray[i] + change * MIN_DIST; plain < dim[i] && plain >= 0; plain += change){
            double t = (plain - ray[i]) / ray[i + 3];
            if (!add_hit(t, ray)){
                break;
            }
        }
    }
    */

    
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
        if (index[i] >= dim[i] || index[i] <= 0){
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
/*
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
*/
}

void Voxel::filter(){

    cv::Mat filter = cv::Mat(dim[1], dim[0], CV_8U, cv::Scalar(255));
    cv::Mat conf(dim[1], dim[0], CV_32FC1);
    //ROS_INFO("filter %i", dim[2]);
    for(int x = 0; x < dim[0]; x++){
        for(int y = 0; y < dim[1]; y++){
            int max = 0;
            int index = dim[2];
            for(int z = 0; z < dim[2]; z++){
            //ROS_INFO("index %i %i %i", x, y, z);
                int nr = grid[x + dim[0] * (y + dim[1] * z)];
            //ROS_INFO("%i %i %i", x,y,z);
                if (nr > max){
                    max = nr;
                    index = z;
                }
            }
            //ROS_INFO("1");
            filter.at<uchar>(y,x) = (uchar) dim[2] - index;
            conf.at<float>(y,x) = (float) max;
            //ROS_INFO("2");
        }

    }
    cv::Mat conf_8;
    cv::normalize(conf, conf_8, 0.0, 255.0, cv::NORM_MINMAX);
    conf_8.convertTo(conf_8, CV_8U);
    cv::Mat mask;
    cv::adaptiveThreshold(conf_8, mask, 1, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 5, -4.0);

    MEDFIL::MedianFilter m;
    m.median(filter, max_dist, mask, 5);

    for (int x = 0; x < dim[0]; x++){
        for(int y = 0; y < dim[1]; y++){
            if (max_dist.at<uchar>(y,x) == 0)
                max_dist.at<uchar>(y,x) = (uchar) 255;
        }
    }
    

    ROS_INFO("1");
/*
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
*/
}

int Voxel::filtered_mark(int w, int h){
    return (int) max_dist.at<uchar>(h,w);
}

void Voxel::depth_map(cv::Mat& img){
    cv::Mat dm = (max_dist) * (255 / (dim[2]));
    img = dm.clone();
}

void Voxel::npy(){
    cnpy::npy_save(std::string("dsi.npy"), &grid[0], {(unsigned long int) dim[2], (unsigned long int) dim[1], (unsigned long int) dim[0]}, "w");
}

} //namespace
