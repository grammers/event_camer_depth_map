#pragma once
#include "ros/ros.h"
#include <opencv2/core/core.hpp>

#define FORWARD 1
#define BACKWARD (-1)

namespace MEDFIL{
class MedianFilter{
public:
    void median(const cv::Mat& img, cv::Mat& filter, const cv::Mat& mask, int patch_size);

private:
    void histogram(const cv::Mat& img, const cv::Mat& mask, int *h, int& num_elements, int row, int col, int p);
    int get_value(const cv::Mat& img, const cv::Mat& mask, int row, int col);
    int compute_median(int *h, int num_elements);

};

} //namespace

