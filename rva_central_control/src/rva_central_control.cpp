#include <ros/ros.h>
#include <ros/console.h>

#include <math.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

//Custom message
#include <recognize_with_vgg/Target_Recognized_Object.h>
#include <rva_central_control/target.h>

// dynamic configure
#include <dynamic_reconfigure/server.h>
#include <rva_central_control/globalParamConfig.h>

#include <stdio.h>

#include "param.h"
#include "request.h"

using namespace std;

typedef recognize_with_vgg::Target_Recognized_Object TRO;

// global value
int OFFSET = 101; // camera pitch offset
double GH = 1.2; // camera base to robot base

bool centralSwitch_ = true; // main switch


// target properties
enum TargetType{t_null, t_onTable, t_onGround, t_onHead, t_onHand};
string targetTypeName[5] = {"in air", "on the table", "on the ground", "on the face", "in the hand"};
int tgtType_ = t_null;
string param_target_type = "/status/target/type";

// target status control elements
string targetLabel_ = "";
bool isTargetSet_ = false;


//target status feedback elements
bool foundTarget_ = false;


// mode control elements
enum ModeType{m_wander, m_recognize, m_search, m_track};
int modeType_ = m_wander;
int modeTypeTemp_ = -1;
string modeName[4] = {"wandering", "recognition", "search", "tracking"};
string param_running_mode = "/status/running_mode";

// mode status  feedback elements
bool recognizeActive_ = false;
bool searchActive_ = false;
bool trackActive_ = false;


bool foundGround_ = false;
bool foundTable_ = false;
bool needSearch_ = false;


void targetCallback(const rva_central_control::targetConstPtr &msg)
{
    if (msg->label.data == " " || msg->label.data == "")
        {
            ROS_WARN("Target canceled.\n");
            tgtType_ = t_null;
            targetLabel_ = "";
            isTargetSet_ = false;
        }
    else
        {
            if (msg->isOnTable.data)
                {
                    tgtType_ = t_onTable;
                }
            else
                {
                    tgtType_ = t_onGround;
                }

            ROS_WARN("Target set to be '%s'.\n", msg->label.data.c_str());
            targetLabel_ = msg->label.data;

            isTargetSet_ = true;
ros::NodeHandle pnh("~");
            ros::param::set(param_target_type, tgtType_);
            ros::param::set("/target/label", targetLabel_);
        }
}

//void searchCallback(const std_msgs::BoolConstPtr &msg)
//{
//    if (msg->data)
//        {
//            needSearch_ = false;
//        }
//}

void getSurfaceCallback(const std_msgs::Int8ConstPtr & msg)
{
    if (modeType_ != m_track)
        {
            switch (msg->data) {
                case 0:
                    // nothing found
                    ROS_INFO_THROTTLE(19, "Neither ground or table is found, maybe robot is shaking or in corner.\n");
                    needSearch_ = true;
                    break;

                case 1:
                    ROS_INFO_THROTTLE(19, "Found table.\n");
                    if (tgtType_ != t_onTable)
                        {
                            needSearch_ = true;
                        }
                    else
                        {
                            needSearch_ = false;
                        }
                    break;

                case 2:
                    ROS_INFO_THROTTLE(19, "Found ground.\n");
                    if (tgtType_ != t_onGround)
                        {
                            needSearch_ = true;
                        }
                    else
                        {
                            needSearch_ = false;
                        }
                    break;

                case 3:
                    ROS_INFO_THROTTLE(19, "Found both table and ground.\n");
                    needSearch_ = false;
                    break;
                }
        }
}


void getObjectCallback(const std_msgs::Int8ConstPtr & msg)
{
    if (modeType_ != m_track)
        {
            switch (msg->data)
                {
                case 0:
                    ROS_INFO_THROTTLE(17,"Nothing found %s.\n", targetTypeName[tgtType_].c_str());
                    needSearch_ = true;
                    break;

                case 1:
                    ROS_INFO_THROTTLE(17,"Found something %s.\n", targetTypeName[tgtType_].c_str());
                    needSearch_ = false;
                    break;
                }
        }
}

void recVGGCallback(const std_msgs::BoolConstPtr & msg)
{
    if (modeType_ == m_recognize)
        {
            if (!msg->data)
                {
//                    foundTarget_ = false;
                }
        }
}

void recConCallback(const std_msgs::BoolConstPtr &msg)
{
    if (modeType_ == m_recognize && !foundTarget_)
        {
            if (!msg->data)
                {
                    ROS_WARN_THROTTLE(23,"Recognization didn't find target.\n");
                    foundTarget_ = false;
                }
            else
                {
                    ROS_WARN_THROTTLE(23,"Target '%s' is found %s!\n", targetLabel_.c_str(), targetTypeName[tgtType_].c_str());
                    foundTarget_ = true;
                }
        }
}

void trackCallback(const std_msgs::BoolConstPtr &msg)
{
    if (modeType_ == m_track)
        {
            if (!msg->data)
                {
                    foundTarget_ = false;
                }
            else
                {
                    ROS_INFO_THROTTLE(19,"Tracking the target...\n");
                    foundTarget_ = true;
                }
        }
}

void dyCallback(rva_central_control::globalParamConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: offset: %d, height: %f.\n", config.camera_pitch_offset_cfg, config.ground_to_base_height_cfg);
    OFFSET = config.camera_pitch_offset_cfg;
    GH = config.ground_to_base_height_cfg;
    // set global params
    ros::param::set("camera_pitch_offset", OFFSET);
    ros::param::set("ground_to_base_height", GH);
}

void resetStatus()
{
		tgtType_ = t_null;
		targetLabel_ = "";
		isTargetSet_ = false;

		needSearch_ = false;

		modeType_ = m_wander;
}

int main(int argc, char **argv)
{
		ros::init(argc, argv, "rva_central_control");

		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");

		pnh.param("camera_pitch_offset", OFFSET, OFFSET);
		pnh.param("ground_to_base_height", GH, GH);

		// set up dynamic reconfigure callback
		dynamic_reconfigure::Server<rva_central_control::globalParamConfig> server;
		dynamic_reconfigure::Server<rva_central_control::globalParamConfig>::CallbackType f;

		f = boost::bind(&dyCallback, _1, _2);
		server.setCallback(f);

		// don't change the order without reason
		ros::Subscriber sub_tgt = nh.subscribe<rva_central_control::target>("recognize/target", 1, targetCallback);
		//\		ros::Subscriber sub_sh = nh.subscribe<std_msgs::Bool>("status/search/feedback", 1, searchCallback);
		ros::Subscriber sub_gs = nh.subscribe<std_msgs::Int8>("status/get/surface/feedback", 1, getSurfaceCallback);
		ros::Subscriber sub_go = nh.subscribe<std_msgs::Int8>("status/get/object/feedback", 1, getObjectCallback);
		ros::Subscriber sub_rs = nh.subscribe<std_msgs::Bool>("status/recognize/vgg/feedback", 1, recVGGCallback);
		ros::Subscriber sub_cs = nh.subscribe<std_msgs::Bool>("status/recognize/confirm/feedback", 1, recConCallback);
		ros::Subscriber sub_tk = nh.subscribe<std_msgs::Bool>("status/track/feedback", 1, trackCallback);

		ROS_WARN("Robot vision system initialized!\n");

		while (ros::ok())
				{
						// main on/off control
						if (ros::param::has("/status/central/switch"))
								{
										bool temp = true;
										ros::param::get("/status/central/switch", temp);
										if (temp)
												{
														ROS_WARN_COND(!centralSwitch_, "Central switch is ON.\n");
												}
										centralSwitch_ = temp;
								}

						if (!centralSwitch_)
								{
										ROS_WARN_THROTTLE(31, "Central switch is OFF.\n");
										resetStatus();
										continue;
								}

						// target infomation preparation before mode selection
						if (sub_tgt.getNumPublishers() == 0)
								{
										ROS_INFO_THROTTLE(11, "Target not set (no target publisher).\n");
										resetStatus();
								}

						needSearch_ = false;

						ros::spinOnce();

						//mode selection, NOTICE that modeType_ should only be set by central control
						if (isTargetSet_)
								{
										if (foundTarget_)
												{
														modeType_ = m_track;
												}
										else
												{
														if (modeType_ != m_search)
																{
																		if (needSearch_)
																				{
																						modeType_ = m_search;
																				}
																		else
																				{
																						modeType_ = m_wander;
																				}
																}
														else
																{
																		modeType_ = m_wander;
																}
												}
										if (modeType_ != m_track && modeType_ != m_search)
												{
														modeType_ = m_recognize;
												}
								}
						else
								{
										modeType_ = m_wander;
								}

						// set mode
						ros::param::set(param_running_mode, modeType_);
						if (modeType_ != modeTypeTemp_)
								{
										ROS_WARN("Current mode: %s.\n", modeName[modeType_].c_str());
										modeTypeTemp_ = modeType_;
								}
				}

		return 0;
}
