#include "quaternion.hpp"

namespace QUAT{
Quaternion::Quaternion(){
    this->w = 1;
    this->x = 0;
    this->y = 0;
    this->z = 0;

    create_rotation();
}
Quaternion::Quaternion(double w, double x, double y, double z){
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;

    create_rotation();

}

Quaternion::Quaternion(Quaternion *q){
    this->w = q->w;
    this->x = q->x;
    this->y = q->y;
    this->z = q->z;

    create_rotation();
}

Quaternion::Quaternion(Quaternion *q1, Quaternion *q2) : Quaternion(q2){
    //q1->invers();
    //this->multiply(q1);
    //q1->invers();

    create_rotation();
    this->r = this->r * q1->r.transpose(); 
}

void Quaternion::set_rot_to_frame(Quaternion *q){
    this->r = this->r * q->r.transpose(); 
}

void Quaternion::create_rotation(){
    //ROS_INFO("%f %f", y, POW(y));
    r << 1 - 2*POW(y) - 2*POW(z),   2*x*y - 2*z*w,              2*x*z + 2*y*w,
         2*x*y + 2*z*w,             1 - 2*POW(x) - 2*POW(z),    2*y*z - 2*x*w,
         2*x*z - 2*y*w,             2*y*z + 2*x*w,              1 - 2*POW(x) - 2*POW(y);
}

void Quaternion::update(Quaternion *q){
    this->w = q->w;
    this->x = q->x;
    this->y = q->y;
    this->z = q->z;

    create_rotation();
}

void Quaternion::update(double w, double x, double y, double z){
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;

    create_rotation();
}

void Quaternion::cross(double *cross_v, Quaternion *q){
    cross_v[0] = q->y * z - y * q->z;
    cross_v[1] = q->z * x - z * q->x;
    cross_v[2] = q->x * y - y * q->z;

}

double Quaternion::dot(Quaternion *q){
    return q->x * x + q->y * y + q->z * z; 
}

void Quaternion::multiply(Quaternion *q){
    double cross_v [3];
    cross(cross_v, q);
    double thata = q->w * w - dot(q); 
    double q1 = q->w * x + w * q->x + cross_v[0];
    double q2 = q->w * y + w * q->y + cross_v[1];
    double q3 = q->w * z + w * q->z + cross_v[2];

    this->w = thata;
    this->x = q1;
    this->y = q2;
    this->z = q3;

    create_rotation();
}

void Quaternion::invers(){
    x = -x;
    y = -y;
    z = -z;
    
    create_rotation();
}

void Quaternion::pow(double n){
    double theta = n * std::acos(w);
    w = std::cos(theta);
    x = x * std::sin(theta);
    y = y * std::sin(theta);
    z = z * std::sin(theta);

    create_rotation();
}

void Quaternion::rotate(double *dir){
    Eigen::Vector3f t;
    t << dir[0], dir[1], dir[2];
    t = r * t;

    for(int i = 0; i < 3; i++)
        dir[i] = t(i);
}

void Quaternion::translate(double *pos_relative, double *pos_origin, double *pos_last){
    Eigen::Matrix4f T;
    Eigen::Vector4f p;
    Eigen::Vector3f t;
    t << pos_origin[0], pos_origin[1], pos_origin[2];
    t = -r.transpose() * t;
    T << r(0,0), r(1,0), r(2,0), t(0),
         r(0,1), r(1,1), r(2,1), t(1), 
         r(0,2), r(1,2), r(2,2), t(2), 
         0, 0, 0, 1; 
    p << pos_last[0], pos_last[1], pos_last[2], 1;

    Eigen::Vector4f new_p = T * p;

    for(int i = 0; i < 3; i++)
        pos_relative[i] = new_p(i);
}

}// namespace
