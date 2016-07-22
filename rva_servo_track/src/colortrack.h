#ifndef COLORTRACK_H
#define COLORTRACK_H

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <vector>

#include <opencv/highgui.h>
#include <opencv/cv.h>

#include "utilities.h"

class ColorTrack
{
public:
    ColorTrack();
    int h_, s_, v_;

    bool process(cv::Mat img_in, cv::Mat &img_out, int &x_out, int &y_out);

private:

};

#endif // COLORTRACK_H
