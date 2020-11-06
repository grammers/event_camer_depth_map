#include "quaternion.hpp"

namespace QUART{
Quarternion::Quarternion(){
    this->w = 1;
    this->x = 0;
    this->y = 0;
    this->z = 0;
}
Quarternion::Quarternion(double w, double x, double y, double z){
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;

}
Quarternion::Quarternion(Quarternion *q){
    this->w = q->w;
    this->x = q->x;
    this->y = q->y;
    this->z = q->z;
}

void Quarternion::update(Quarternion *q){
    this->w = q->w;
    this->x = q->x;
    this->y = q->y;
    this->z = q->z;
}

void Quarternion::update(double w, double x, double y, double z){
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
}

void Quarternion::cross(double *cross_v, Quarternion *q){
    cross_v[0] = q->y * z - y * q->z;
    cross_v[1] = q->z * x - z * q->x;
    cross_v[2] = q->x * y - y * q->z;

}

double Quarternion::dot(Quarternion *q){
    return q->x * x + q->y * y + q->z * z; 
}

void Quarternion::multiply(Quarternion *q){
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
}

void Quarternion::invers(){
    x = -x;
    y = -y;
    z = -z;
}

void Quarternion::pow(double n){
    double theta = n * std::acos(w);
    w = std::cos(theta);
    x = x * std::sin(theta);
    y = y * std::sin(theta);
    z = z * std::sin(theta);
}

}// namespace
