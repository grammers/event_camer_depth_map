#pragma once
# include "ros/ros.h"
#include <Eigen/Core>

#define POW(x) ((x)*(x))

namespace QUAT{
class Quaternion{
public:
    double w;
    double x;
    double y;
    double z;

    Eigen::Matrix3f r;

    Quaternion();
    Quaternion(double w, double x, double y, double z);
    Quaternion(Quaternion *q);
    Quaternion(Quaternion *q1, Quaternion *q2);

    void set_rot_to_frame(Quaternion *q);
    void update(Quaternion *q);
    void update(double w, double x, double y, double z);
    void multiply(Quaternion *q);
    void pow(double n);
    void invers();
    void translate(double *pos_relative, double *pos_origin, double *pos_last);
    void rotate(double *dir);


private:
    void create_rotation();
    double dot(Quaternion *q);
    void cross(double *cross_v, Quaternion *q);

};


} //name space
