#pragma once
# include "ros/ros.h"

namespace QUART{
class Quarternion{
public:
    double w;
    double x;
    double y;
    double z;

    Quarternion();
    Quarternion(double w, double x, double y, double z);
    Quarternion(Quarternion *q);
    void update(Quarternion *q);
    void update(double w, double x, double y, double z);
    void multiply(Quarternion *q);
    void pow(double n);
    void invers();


private:
    double dot(Quarternion *q);
    void cross(double *cross_v, Quarternion *q);

};


} //name space
