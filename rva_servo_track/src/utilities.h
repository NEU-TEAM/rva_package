#ifndef UTILITIES_H
#define UTILITIES_H

#include <opencv2/opencv.hpp>

class Utilities
{
public:
    Utilities();

    static void drawObject(int x, int y, cv::Mat &frame);
    static std::string intToString(int number);

};

#endif // UTILITIES_H
