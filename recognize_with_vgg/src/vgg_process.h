#ifndef VGG_PROCESS_H
#define VGG_PROCESS_H

#include <cuda_runtime.h>

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <vector>

#include <opencv/highgui.h>
#include <opencv/cv.h> //cvResize()

#include "caffe/caffe.hpp"

using namespace caffe;  // NOLINT(build/namespaces)
using namespace std;

#define IMAGE_SIZE 224

class vgg_process
{
public:
    vgg_process();
    bool process(cv::Mat image_in, string &name, float &conf);

private:
    void get_top5(float *feature, int arr[5]);
    void bubble_sort(float *feature, int *sorted_idx);
    void get_label(char filename[256], char label[][512]);

    // read labels
    char label[1000][512];
};

#endif // VGG_PROCESS_H
