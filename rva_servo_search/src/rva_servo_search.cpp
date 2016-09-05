#include <ros/ros.h>

#include <math.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt16MultiArray.h>

#include "search.h"

using namespace std;

ros::Publisher servoPubSearch_;
ros::Publisher searchPubStatus_;

enum ModeType{m_wander, m_recognize, m_search, m_track};
int modeType_ = m_wander;
string param_running_mode = "/status/running_mode";

bool noResult_ = false; // feedback

int main(int argc, char **argv)
{
		ros::init(argc, argv, "rva_servo_search");

		ros::NodeHandle nh;

		servoPubSearch_ = nh.advertise<std_msgs::UInt16MultiArray>("servo", 1);
		searchPubStatus_ = nh.advertise<std_msgs::Bool>("status/search/feedback", 1);

		ROS_WARN("Search function initialized!\n");

		Search sh;

		ros::Rate loop_rate(0.2); //0.2hz
		while (ros::ok())
				{
						if (ros::param::has(param_running_mode))
								ros::param::get(param_running_mode, modeType_);

						if (modeType_ != m_search)
								{
										continue;
								}

						int yawAngle = 90;
						int pitchAngle = 70;
						noResult_ = sh.getNextPosition(yawAngle, pitchAngle);
						ROS_WARN("Searching with yaw %d, pitch %d.\n", yawAngle, pitchAngle);
						if (noResult_)
								{
										ROS_WARN("Searching around didn't find target.\n");
										sh.getCenter(yawAngle, pitchAngle);
										sh.resetSearch();
								}
						std_msgs::UInt16MultiArray array;
						array.data.push_back(pitchAngle);
						array.data.push_back(yawAngle);
						servoPubSearch_.publish(array);

						std_msgs::Bool flag;
						flag.data = true;
						searchPubStatus_.publish(flag);

						loop_rate.sleep();
//						ros::spinOnce();
				}

		return 0;
}
