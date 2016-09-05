#include <ros/ros.h>

#include <math.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16MultiArray.h>

//Custom message
#include <rva_central_control/target.h>
#include <recognize_with_vgg/Target_Recognized_Object.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include "colortrack.h"
#include "camshift.h"

float TOANGLEX = 0.02; // a reasonable speed
float TOANGLEY = 0.02;

using namespace std;

int pitch_ = 70;
int yaw_ = 90;

// wait loop for target lost
#define WAIT_LOOP 50
int delay_ = WAIT_LOOP;

typedef recognize_with_vgg::Target_Recognized_Object TRO;

image_transport::Publisher imagePubTrack_ ;
ros::Publisher servoPubTrack_;
ros::Publisher trackPubStatus_;
ros::Publisher trackPubTRO_; // notice the topic is different from the subscribed one

cv_bridge::CvImageConstPtr src_;
cv::Mat src_img_;

cv_bridge::CvImagePtr track_ptr_(new cv_bridge::CvImage());
cv::Mat track_image_;

enum ModeType{m_wander, m_recognize, m_search, m_track};
int modeType_ = m_wander;
int modeTypeTemp_ = m_wander;

bool servo_initialized_ = false;

string param_running_mode = "/status/running_mode";
bool isInTracking_ = true;// cause the first call for tracking means have something to track

//ColorTrack CT;
CamShift CS;

std_msgs::String tgt_label_;
cv::Rect detection_;
geometry_msgs::Quaternion support_plane_;

void servoCallback(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
    // this callback should always active
    pitch_ = msg->data[0];
    yaw_ = msg->data[1];
}

void resultCallback(const TRO::ConstPtr & msg)
{
    tgt_label_ = msg->labels;
    int minh = msg->boundaries.boundaries[0];
    int maxh = msg->boundaries.boundaries[1];
    int minw = msg->boundaries.boundaries[2];
    int maxw = msg->boundaries.boundaries[3];

    // omit the background
    detection_ = cv::Rect(minw+10, minh+10, maxw - minw - 20, maxh - minh - 20);

    support_plane_ = msg->support_plane;
}

void publishServo(int pitch_angle, int yaw_angle)
{
    std_msgs::UInt16MultiArray array;
    array.data.push_back(pitch_angle);
    array.data.push_back(yaw_angle);
    pitch_ = pitch_angle;
    yaw_ = yaw_angle;
    servoPubTrack_.publish(array);
}

void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
		if (modeType_ != m_track)
				{
						return;
				}

    if (image_msg->height != 480)
        {
            ROS_ERROR_THROTTLE(5,"Image size is wrong.\n");
            return;
        }

    src_ = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
    src_img_ = src_->image;
    src_img_.copyTo(track_image_);

//    cv::rectangle(track_image_, detection_, cv::Scalar(232,228,53),2);

    cv::RotatedRect roi;

    // check target area
    // CT.process(src_img_, track_image_, xpos, ypos)
    if (detection_.area() < 400)
        {
            isInTracking_ = false;
            ROS_WARN("Target area in image is %d, too small to be tracked.\n", detection_.area());

            CS.tracker_initialized = false;
        }

    std::vector<int> mask_id; // store object pixels id in image
    if (CS.process(src_img_,detection_, track_image_, roi, mask_id))
        {
            // publish exact location and boundaries of tracked object
            track_ptr_->header = image_msg->header;
            track_ptr_->image = track_image_;
            track_ptr_->encoding = sensor_msgs::image_encodings::BGR8;
            imagePubTrack_.publish(track_ptr_->toImageMsg());

            //servo make camera central on object
            int d_x = roi.center.x - 320;
            int d_y = roi.center.y - 240;
            int deg_x = int(d_x * TOANGLEX); // offset the robot head need to turn
            int deg_y = int(d_y * TOANGLEY);

            if (abs(deg_x) < 2 && abs(deg_y) < 1)
                {
                    isInTracking_ = true;
                    delay_ = WAIT_LOOP;

                    // publish new TRO info
                    TRO result;
                    result.header = image_msg->header;
                    result.labels = tgt_label_;
                    result.objects_pixels.vector_pixels = mask_id;
                    result.support_plane = support_plane_;

                    trackPubTRO_.publish(result);

                    return;
                }

            // make the head don't turn too fast
            if (deg_x < -4) deg_x = -4;
            if (deg_x > 4) deg_x = 4;
            if (deg_y < -4) deg_y = -4;
            if (deg_y > 4) deg_y = 4;

            int x_ang = - deg_x + yaw_;
            int y_ang = - deg_y + pitch_;
            if (x_ang >= 0 && x_ang <= 180 && y_ang > 40 && y_ang < 120)
                {
                    isInTracking_ = true;
                    delay_ = WAIT_LOOP;

                    publishServo(y_ang, x_ang);
                }
            else
                {
                    isInTracking_ = false;
                    ROS_WARN("Target out of range.\n");

                    CS.tracker_initialized = false;
                }
        }
    else
        {
            delay_--;
            if (delay_ < 0)
                {
                    isInTracking_ = false;
                    ROS_WARN("Target lost.\n");

                    CS.tracker_initialized = false;
                }
        }
}

int main(int argc, char **argv)
{
		ros::init(argc, argv, "rva_servo_track");

		ros::NodeHandle nh;

		image_transport::ImageTransport it(nh);

		imagePubTrack_ = it.advertise("track/image", 1);
		servoPubTrack_ = nh.advertise<std_msgs::UInt16MultiArray>("servo", 1, true);
		trackPubStatus_ = nh.advertise<std_msgs::Bool>("status/track/feedback", 1);
		trackPubTRO_ = nh.advertise<TRO>("track/confirm/result" , 1);

		ros::Subscriber sub_res = nh.subscribe<TRO>("recognize/confirm/result", 1, resultCallback);
		image_transport::Subscriber sub_rgb = it.subscribe("/rgb/image", 1, imageCallback);
		ros::Subscriber sub_s = nh.subscribe<std_msgs::UInt16MultiArray> ("servo", 1, servoCallback);

		ROS_WARN("Tracking function initialized!\n");

		while (ros::ok())
				{
						if (ros::param::has(param_running_mode))
								{
										ros::param::get(param_running_mode, modeTypeTemp_);

										if (modeTypeTemp_ != m_track && modeType_ == m_track)
												{
														CS.tracker_initialized = false;
												}
										modeType_ = modeTypeTemp_;
								}

						std_msgs::Bool flag;
						flag.data = true;

						if (!servo_initialized_)
								{
										publishServo(70, 90);
										servo_initialized_ = true;
								}

						ros::spinOnce();

						flag.data = isInTracking_;
						trackPubStatus_.publish(flag);
				}

		return 0;
}

