#include <ros/ros.h>

#include <math.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>

//Custom message
#include <cloud_get_object/Objects_Pixels_Vector.h>
#include <cloud_get_object/Object_Image_Pixels.h>
#include <rva_central_control/target.h>

#include <recognize_with_vgg/VGG_Recognized_Object.h>
#include <recognize_with_vgg/Target_Recognized_Object.h>

using namespace std;

typedef recognize_with_vgg::VGG_Recognized_Object VRO;
typedef recognize_with_vgg::Target_Recognized_Object TRO;
typedef rva_central_control::target TGT;

ros::Publisher recognizePubResult_;
ros::Publisher recognizePubStatus_;

string targetLabel_;

enum ModeType{m_wander, m_recognize, m_search, m_track};
int modeType_ = m_wander;
string param_running_mode = "/status/running_mode";

bool isConfirmed_ = false;

void resultCallback(const VRO::ConstPtr& result_msg)
{
    if (result_msg->labels.empty())
        {
            ROS_INFO("Found 0 object.\n");
            isConfirmed_ = false;
            return;
        }

    TRO result;

    for (size_t i = 0; i < result_msg->labels.size(); i++)
        {
            result.header = result_msg->header;
            std_msgs::String str;
            string labeldata = result_msg->labels[i].data;

            if (labeldata == targetLabel_)
                {
                    cerr << "&Found target object: " << targetLabel_ << "." << endl;

                    str.data = targetLabel_;
                    result.labels = str;
                    result.boundaries = result_msg->boundaries[i];
                    result.objects_pixels = result_msg->objects_vector.vector_objects[i];
                    result.hsv = result_msg->hsv_vector[i];

                    isConfirmed_ = true;
                    break;
                }
            else
                {
                    cerr << "Found  " << labeldata << ", but not target object." << endl;
                    isConfirmed_ = false;
                }
        }

    if (isConfirmed_)
        recognizePubResult_.publish(result);
}

int main(int argc, char **argv)
{
		ros::init(argc, argv, "io_confirm_object");

		ros::NodeHandle nh;

		recognizePubResult_ = nh.advertise<TRO>("recognize/confirm/result", 1, true);
		recognizePubStatus_ = nh.advertise<std_msgs::Bool>("status/recognize/confirm/feedback", 1);

		ros::Subscriber sub_vro = nh.subscribe<VRO>("recognize/vgg/result", 1, resultCallback);

		while (ros::ok())
				{
						if (ros::param::has(param_running_mode))
								ros::param::get(param_running_mode, modeType_);

						if (ros::param::has("/target/label"))
								ros::param::get("/target/label", targetLabel_);

						std_msgs::Bool flag;

						if (modeType_ != m_recognize)
								{
										ROS_INFO("Confirm recognize result is not actived.\n");
										continue;
								}

						ros::spinOnce();

						flag.data = isConfirmed_;
						recognizePubStatus_.publish(flag);
				}

		return 0;
}
