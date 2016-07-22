#ifndef CAMSHIFT_H
#define CAMSHIFT_H

#include <opencv2/opencv.hpp>

#include "utilities.h"

class CamShift
{
public:
    CamShift();

    cv::RotatedRect camshift_init (cv::Mat img, cv::Rect detection);
    // This version uses the last detection as initializing rectangle.
    cv::RotatedRect camshift_track (cv::Mat img);

    bool process(cv::Mat img_in, cv::Rect detection, cv::Mat &img_out, int &center_x, int &center_y, cv::RotatedRect &roi);

    bool initialized();

    bool tracker_initialized;

private:
    cv::Mat hist_;
    cv::RotatedRect last_trackbox_;
    cv::Rect last_detection_;

    int hue_size_, sat_size_, sat_min_, val_min_, val_max_;
    float hue_range_[2];
    float sat_range_[2];
    const float* pranges_[2];
};

#endif // CAMSHIFT_H
