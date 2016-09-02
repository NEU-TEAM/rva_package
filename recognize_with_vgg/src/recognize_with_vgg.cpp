#include <ros/ros.h>

#include <math.h>
#include <vector>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16MultiArray.h>

//Custom message
#include <cloud_get_object/Objects_Pixels_Vector.h>
#include <cloud_get_object/Object_Image_Pixels.h>
#include <recognize_with_vgg/Object_Pixels_Boundaries.h>
#include <recognize_with_vgg/VGG_Recognized_Object.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include "vgg_process.h"
#include "utilities.h"

typedef cloud_get_object::Object_Image_Pixels OIP;
typedef cloud_get_object::Objects_Pixels_Vector OPV;
typedef recognize_with_vgg::Object_Pixels_Boundaries OPB;
typedef recognize_with_vgg::VGG_Recognized_Object VRO;

image_transport::Publisher imagePubLabel_ ;
ros::Publisher recognizePubInfo_;
ros::Publisher statusPubRec_;

cv_bridge::CvImageConstPtr src_;
cv::Mat src_img_;

cv_bridge::CvImagePtr labeled_ptr_(new cv_bridge::CvImage());
cv::Mat labeled_img_;

enum ModeType{m_wander, m_recognize, m_search, m_track};
int modeType_ = m_wander;
string param_running_mode = "/status/running_mode";

bool recognizedThings_ = false;

//vgg_process vgg ;

bool resolvePosition(std::vector<int> obj_pos, int &minh, int &maxh, int &minw, int &maxw)
{
    minh = 480;
    maxh = 0;
    minw = 640;
    maxw = 0;
    for (size_t i = 0; i < obj_pos.size(); i++)
        {
            int val = obj_pos[i];
            if (val > 307199 || val < 0)
                return false;

            int h = val / 640;
            int w = val - (val / 640) * 640;
            if (h < minh)
                {
                    minh = h;
                }
            if (h > maxh)
                {
                    maxh = h;
                }

            if (w > maxw)
                {
                    maxw = w;
                }
            if (w < minw)
                {
                    minw = w;
                }
        }
    return true;
}

bool prepareImage(cv::Mat src_img, cv::Mat &out_image, std::vector<int> obj_pos, int minh, int maxh, int minw, int maxw)
{
    int range_row = maxh - minh;
    int range_col = maxw - minw;

    int range;
    int midh = (minh+maxh)/2;
    int midw = (minw+maxw)/2;
    if (range_row > range_col)
        {
            out_image = cv::Mat(cv::Size(range_row, range_row), CV_8UC3, cv::Scalar(104, 117, 124));
            range = range_row;
        }
    else
        {
            out_image = cv::Mat(cv::Size(range_col, range_col), CV_8UC3, cv::Scalar(104, 117, 124));
            range = range_col;
        }

    if (midh < range/2 || midw < range/2 || src_img.rows - midh < range/2 || src_img.cols - midw < range/2)
        return false;

    for (size_t i = 0; i < range; i++)
        {
            for (size_t j = 0; j < range; j++)
                {
                            cv::Vec3b intensity = src_img.at<cv::Vec3b>(midh - range/2 + i, midw - range/2 + j);
                            out_image.at<cv::Vec3b>(i, j) = intensity;
                }
        }
    return true;

//    for (size_t i = 0; i < obj_pos.size(); i++)
//        {
//            int val = obj_pos[i];
//            if (val > 307199 || val < 0)
//                return;

//            int h = val / 640;
//            int w = val - (val / 640) * 640;

//            if (h - minh < range_row && w - minw < range_col )
//                {
//                    cv::Vec3b intensity = src_img.at<cv::Vec3b>(h, w);
//                    out_image.at<cv::Vec3b>(h - minh, w - minw) = intensity;
//                }
//        }

//    int offset = 10;//maxmize image leftup point by 10X10
//    for (size_t i = 0; i < range; i++)
//        {
//            for (size_t j = 0; j < range; j++)
//                {
//                    if (minh + i < src_img.rows  &&  minw + j < src_img.cols )
//                        {
//                            cv::Vec3b intensity = src_img.at<cv::Vec3b>(minh + i, minw + j);
//                            out_image.at<cv::Vec3b>(i, j) = intensity;
//                        }
//                }
//        }
}


void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    if (image_msg->height != 480)
        {
            ROS_ERROR_THROTTLE(5,"Can't get image.\n");
            recognizedThings_ = false;
            return;
        }
    src_ = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);

    src_img_ = src_->image;
    src_img_.copyTo(labeled_img_);
}

void resolveColor(cv::Mat img, std::vector<int> vec, int &h, int &s, int &v)
{
		h = 0;
		s = 0;
		v = 0;
		cv::Mat HSV;
		//convert frame from BGR to HSV colorspace
		cv::cvtColor(img,HSV,cv::COLOR_BGR2HSV);
		for (size_t i = 0; i < vec.size(); i++)
				{
						cv::Vec3b intensity = HSV.at<cv::Vec3b>(i / 640, i % 640);
						h = (intensity[0] + h) / 2;
						s = (intensity[1] + s) / 2;
						v = (intensity[2] + v) / 2;
				}
}

void labelCallback(const OPV::ConstPtr& pixel_msg)
{
    if (pixel_msg->vector_objects.empty())
        {
            ROS_ERROR_THROTTLE(5,"Can't get object pixel message.\n");
            recognizedThings_ = false;
            return;
        }

    VRO result;
    vgg_process vgg;

    bool detected = false;

    string label_temp = "";
    int minh_temp = 0;
    int maxh_temp = 0;
    int minw_temp = 0;
    int maxw_temp = 0;
    for (size_t i = 0; i < pixel_msg->vector_objects.size(); i++)
        {
            std::vector<int> obj_pos; // object pixels' id in image
            for (size_t j = 0; j < pixel_msg->vector_objects[i].vector_pixels.size(); j++)
                {
                    int idx = pixel_msg->vector_objects[i].vector_pixels[j];
                    obj_pos.push_back(idx);
                }

            int minh, maxh, minw, maxw;

            if  (resolvePosition(obj_pos, minh, maxh, minw, maxw) != true)
                {
                    continue;
                }

            string label;
            cv::Mat square_image;
            if ( !prepareImage(src_img_, square_image, obj_pos, minh, maxh, minw, maxw))
                continue;
//            cv::imshow("s", square_image);
//            cv::waitKey();

            float conf = 0.0;

            detected = vgg.process(square_image, label, conf);
            label.erase(label.end() - 1, label.end()); // remove the last 1 characters, which is '\n'
            if (detected)
                {
                    OPB boundaries; // object pixel boundaries (in image)
                    OIP obj_pixel_id;
                    int min_w, min_h, max_w, max_h;

                    // judge if two objects are overlaied with each other in image
                    bool isOverLay = Utilities::overlay(minh, maxh, minw, maxw,
                                                                                           minh_temp, maxh_temp, minw_temp, maxw_temp,
                                                                                           min_h, max_h, min_w, max_w);
                    if (isOverLay && label == label_temp)
                        {
                            // merge result
                            cv::rectangle(labeled_img_, cv::Rect(min_w, min_h, max_w - min_w, max_h - min_h), cv::Scalar(232,228,53),2);
                            cv::putText(labeled_img_, label, cv::Point((min_w + max_w)/2, (min_h + max_h)/2), cv::FONT_HERSHEY_PLAIN, 1.5,cv::Scalar(30,56,255),2);
                            std::cerr << label << "  recognized with confidence: " << conf << ". (merged)"<< std::endl;

                            // generate image message
                            labeled_ptr_->header = pixel_msg->header;
                            labeled_ptr_->image = labeled_img_;
                            labeled_ptr_->encoding = sensor_msgs::image_encodings::BGR8;
                            imagePubLabel_.publish(labeled_ptr_->toImageMsg());

                            // merge the result
                            std_msgs::String str;
                            str.data = label;
                            result.labels.pop_back();
                            result.labels.push_back(str);

                            boundaries.boundaries.push_back(min_h);
                            boundaries.boundaries.push_back(max_h);
                            boundaries.boundaries.push_back(min_w);
                            boundaries.boundaries.push_back(max_w);
                            result.boundaries.pop_back();
                            result.boundaries.push_back(boundaries);

                            //  todo:   mergeObjPixels(result.objects_vector)
                            result.objects_vector.vector_objects.pop_back();
                            for (size_t k = 0; k < obj_pos.size(); k++)
                                {
                                    obj_pixel_id.vector_pixels.push_back(obj_pos[k]);
                                }
                            result.objects_vector.vector_objects.push_back(obj_pixel_id);

                            int h,s,v;
                            resolveColor(src_img_, obj_pos, h, s, v);
                            std_msgs::UInt16MultiArray hsv;
                            hsv.data.push_back(h);
                            hsv.data.push_back(s);
                            hsv.data.push_back(v);
                            result.hsv_vector.pop_back();
                            result.hsv_vector.push_back(hsv);

                            // save temp data
                            label_temp = label;
                            minh_temp = min_h;
                            maxh_temp = max_h;
                            minw_temp = min_w;
                            maxw_temp = max_w;
                        }
                    else
                        {
                            cv::rectangle(labeled_img_, cv::Rect(minw, minh, maxw - minw, maxh - minh), cv::Scalar(232,228,53),2);
                            cv::putText(labeled_img_, label, cv::Point((minw+maxw)/2, (minh + maxh)/2), cv::FONT_HERSHEY_PLAIN, 1.5,cv::Scalar(30,56,255),2);
                            std::cerr << label << "  recognized with confidence: " << conf << " ! "<< std::endl;

                            // generate image message
                            labeled_ptr_->header = pixel_msg->header;
                            labeled_ptr_->image = labeled_img_;
                            labeled_ptr_->encoding = sensor_msgs::image_encodings::BGR8;
                            imagePubLabel_.publish(labeled_ptr_->toImageMsg());

                            std_msgs::String str;
                            str.data = label;
                            result.labels.push_back(str);

                            // get together the result
                            boundaries.boundaries.push_back(minh);
                            boundaries.boundaries.push_back(maxh);
                            boundaries.boundaries.push_back(minw);
                            boundaries.boundaries.push_back(maxw);
                            result.boundaries.push_back(boundaries);

                            for (size_t k = 0; k < obj_pos.size(); k++)
                                {
                                    obj_pixel_id.vector_pixels.push_back(obj_pos[k]);
                                }
                            result.objects_vector.vector_objects.push_back(obj_pixel_id);

                            int h,s,v;
                            resolveColor(src_img_, obj_pos, h, s, v);
                            std_msgs::UInt16MultiArray hsv;
                            hsv.data.push_back(h);
                            hsv.data.push_back(s);
                            hsv.data.push_back(v);
                            result.hsv_vector.push_back(hsv);

                            // save temp data
                            label_temp = label;
                            minh_temp = minh;
                            maxh_temp = maxh;
                            minw_temp = minw;
                            maxw_temp = maxw;
                        }

                    recognizedThings_ = true;
                }
            else
                {
                    std::cerr << "Recognizing  with minium confidence: 0.3." << std::endl;
                }
        }

    result.header = pixel_msg->header;
    result.support_plane = pixel_msg->support_plane;

    recognizePubInfo_.publish(result);
}

int main(int argc, char **argv)
{
		ros::init(argc, argv, "recognize_with_vgg");

		ros::NodeHandle nh;
		image_transport::ImageTransport it(nh);

		imagePubLabel_ = it.advertise("recognize/vgg/labeled_image", 1);

		recognizePubInfo_ = nh.advertise<VRO>("recognize/vgg/result", 1);
		statusPubRec_ = nh.advertise<std_msgs::Bool>("status/recognize/vgg/feedback", 1);

		image_transport::Subscriber sub_rgb = it.subscribe("/rgb/image", 1, imageCallback);

		ros::Subscriber sub_opv = nh.subscribe<OPV>("get/object/objects_in_image", 1, labelCallback);

		while (ros::ok())
				{
						if (ros::param::has(param_running_mode))
								ros::param::get(param_running_mode, modeType_);

						std_msgs::Bool flag;

						if (modeType_ != m_recognize)
								{
										ROS_INFO_THROTTLE(5,"Recognize is not actived.\n");
										recognizedThings_ = false;
										flag.data = recognizedThings_;
										statusPubRec_.publish(flag);
										continue;
								}

						ros::spinOnce();

						flag.data = recognizedThings_;
						statusPubRec_.publish(flag);
				}

		return 0;
}
