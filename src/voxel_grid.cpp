#include <voxel_grid.hpp>

#define THRES 0.1 //10000 //4500 //10000
//#define THRES 1000 //4500 //10000
#define MIN_DIST 10

namespace GRID{

Voxel::Voxel(int dimX, int dimY, int dimZ){

    dim[0] = dimX;
    dim[1] = dimY;
    dim[2] = dimZ;
    
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



void Voxel::clear(){
    int size = dim[0] * dim[1] * dim[2];
    for (int i = 0; i < size; i++){
        grid[i] = 0;
    }
}


void Voxel::filter(){

    cv::Mat filter = cv::Mat(dim[1], dim[0], CV_8U, cv::Scalar(255));
    cv::Mat conf(dim[1], dim[0], CV_32FC1);
    for(int x = 0; x < dim[0]; x++){
        for(int y = 0; y < dim[1]; y++){
            int max = 0;
            int index = dim[2];
            for(int z = 0; z < dim[2]; z++){
                int nr = grid[x + dim[0] * (y + dim[1] * z)];
                if (nr > max){
                    max = nr;
                    index = z;
                }
            }
            filter.at<uchar>(y,x) = (uchar) dim[2] - index;
            conf.at<float>(y,x) = (float) max;
        }
    }

    cv::Mat conf_8;
    cv::normalize(conf, conf_8, 0.0, 255.0, cv::NORM_MINMAX);
    conf_8.convertTo(conf_8, CV_8U);
    cv::Mat mask;
    cv::adaptiveThreshold(conf_8, mask, 1, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, KERNAL, -ADAPT);

    MEDFIL::MedianFilter m;
    m.median(filter, max_dist, mask, MED_FILTER);

    for (int x = 0; x < dim[0]; x++){
        for(int y = 0; y < dim[1]; y++){
            if (max_dist.at<uchar>(y,x) == 0)
                max_dist.at<uchar>(y,x) = (uchar) 255;
        }
    }
    

    ROS_INFO("1");
}

void Voxel::depth_map(cv::Mat& img){
    cv::Mat dm = (max_dist) * (255 / (dim[2]));
    img = dm.clone();
}

void Voxel::npy(){
    cnpy::npy_save(std::string("dsi.npy"), &grid[0], {(unsigned long int) dim[2], (unsigned long int) dim[1], (unsigned long int) dim[0]}, "w");
}

} //namespace
