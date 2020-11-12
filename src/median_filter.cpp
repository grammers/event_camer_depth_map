#include <median_filter.hpp>

namespace MEDFIL{

int MedianFilter::get_value(const cv::Mat& img, const cv::Mat& mask, int row, int col){
    //ROS_INFO("row col: %i %i", row, col);
    if(row >= 0 && col >= 0 &&
        row < img.rows && col < img.cols &&
        mask.at<uint8_t>(row,col) > 0){

        return img.at<uchar>(row,col);
    }
    else {
        return -1;
    }
    
}

void MedianFilter::histogram(const cv::Mat& img, const cv::Mat& mask, int *h, int& num_elements, int row, int col, int p){
    for(row = -p; row <= p; row++){
        for(col = -p; col <= p; col++){
            int val = get_value(img, mask, row, col);
            if (val >= 0){
                h[val]++;
                num_elements++;
            }
        }
    }

}

int MedianFilter::compute_median(int *h, int num_elements){
    int midle = (num_elements + 1) / 2;
    int m, n;
    for(m = 0, n = 0; n < 256; n++){
        m += h[n];
        if(m >= midle)
            break;
    }
    return n;
}

void MedianFilter::median(const cv::Mat& img, cv::Mat& filter, const cv::Mat& mask, int patch_size){

    //ROS_INFO("arived");
    filter = img.clone();
    //ROS_INFO("clone");
    int p = patch_size / 2;
    int med;
    int prev, next;
    int h[256];
    int direction = FORWARD;
    int row, row1, row2,col, col1, col2, r, c;

    memset(h, 0, sizeof(h));
    int num_elements = 0;
    //ROS_INFO("def var");

    histogram(img, mask, h, num_elements, row, col, p);

    filter.at<uchar>(0,0) = compute_median(h, num_elements);

    //ROS_INFO("pre loop");
    // manin loop
    for(col = 1, row = 0; row < img.rows; row++){
        row1 = row - p;
        row2 = row + p;

        // col loop
        for(; col >= 0 && col < img.cols; col += direction){
            prev = col - direction * (p + 1);
            next = col + direction * p;

            for(r = row1; r <= row2; r++){
                int out = get_value(img, mask, r, prev);
                int in = get_value(img, mask, r, next);

                //if (in >= 0 && out == in){
                //    continue;
                //}
                if (out >= 0){
                    h[out]--;
                    num_elements--;
                }
                if (in >= 0){
                    h[in]++;
                    num_elements++;
                }
            }

            med = compute_median(h, num_elements);
            filter.at<uchar>(row,col) = med;

        } //end col loop

        //if (row == img.rows - 1)
        //    return;

        col -= direction;
        direction *= -1;

        prev = row1;
        next = row2 + 1;
        col1 = col - p;
        col2 = col + p;
        
        for(c = col1; c <= col2; c++){
            int out = get_value(img, mask, prev, c);
            int in = get_value(img, mask, next, c);

            //if (in >= 0 && out == in)
            //    continue;
            if(out >= 0){
                h[out]--;
                num_elements--;
            }
            if (in >= 0){
                h[in]++;
                num_elements++;
            }
        }

        filter.at<uchar>(row + 1, col) = compute_median(h, num_elements);
        col += direction;

    } //end main loop
}

} //namespace

