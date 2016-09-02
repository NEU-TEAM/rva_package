#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16MultiArray.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#define DEBUG_TRANS

// camera frame rotation about world frame
float pitch_ = 0.0; // rotation angle -90 between camera optical frame and link is not included
float yaw_ = 0.0;

// distance between frame orign, in m
float dy_link_to_pitch = 0.00375;
float dz_link_to_pitch = 0.027;

float dx_pitch_to_yaw = 0.019;
float dz_pitch_to_yaw = 0.0773;

double GH;

float pitchtemp_ = 0.0;
int tol_ = 2; // the tolerance (degree) about camera shaking
int OFFSET = 101; // offset for pitch which was got from IMU

// roslaunch param
std::string cameraLinkFrameID_ = "/camera_link_frame";
std::string cameraPitchFrameID_ = "/camera_pitch_frame"; // y along the axis of pitch servo to the right, x toward front, z up, orign projection at the diameter of rotation plate
std::string cameraYawFrameID_ = "/camera_yaw_frame"; // y to the right, x to the front, z up, orign at the bottom of support
std::string pointCloudSourceTopic_ = "/point_cloud";

void pitchCallback(const std_msgs::Float32ConstPtr & msg)
{
    pitch_ = msg->data - OFFSET; // notice that pitch_ < 0
    if (fabs(pitch_ - pitchtemp_) > tol_ )
        pitchtemp_ = pitch_;
    pitch_ = pitchtemp_  * 0.01745; // / 180.0 * 3.14159
}

void yawCallback(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
    // this callback should always active
    yaw_ = (msg->data[1] - 90) * 0.01745;
}

void tfCallback(const std_msgs::HeaderConstPtr & msg)
{
    ROS_WARN_THROTTLE(13, "Broadcasting tf...");

    static tf::TransformBroadcaster br_optic_to_link;
    tf::Transform tf_optic_to_link;

    tf_optic_to_link.setOrigin( tf::Vector3(0, 0.01, 0.02) );
    tf::Quaternion q;
    q.setRPY(3*M_PI_2 , 0, 3*M_PI_2);  // translate the camera link frame into pitch frame
//    q.setRPY(-M_PI/2 , 0, -M_PI/2);  // translate the camera link frame into pitch frame

    tf_optic_to_link.setRotation(q);
    br_optic_to_link.sendTransform(tf::StampedTransform(tf_optic_to_link, msg->stamp, cameraLinkFrameID_, "/openni_rgb_optical_frame"));

    static tf::TransformBroadcaster br_link_to_pitch;
    tf::Transform tf_link_to_pitch;

    tf_link_to_pitch.setOrigin( tf::Vector3(0, dy_link_to_pitch, dz_link_to_pitch) );
    tf::Quaternion q_p;
    q_p.setRPY(0 , pitch_, 0);  // Notice that a negtive value may not locate in the x value of sin(x)
    q_p.setY(-q_p.y());

    tf_link_to_pitch.setRotation(q_p);
    br_link_to_pitch.sendTransform(tf::StampedTransform(tf_link_to_pitch, msg->stamp, cameraPitchFrameID_, cameraLinkFrameID_));


    static tf::TransformBroadcaster br_pitch_to_yaw;
    tf::Transform tf_pitch_to_yaw;

    tf_pitch_to_yaw.setOrigin( tf::Vector3(dx_pitch_to_yaw, 0, dz_pitch_to_yaw) );
    tf::Quaternion q_y;
    q_y.setRPY(0 , 0, yaw_);  // translate the camera pitch frame into yaw frame

    tf_pitch_to_yaw.setRotation(q_y);
    br_pitch_to_yaw.sendTransform(tf::StampedTransform(tf_pitch_to_yaw, msg->stamp,  cameraYawFrameID_, cameraPitchFrameID_));

    static tf::TransformBroadcaster br_yaw_to_base;
    tf::Transform tf_yaw_to_base;

    tf_yaw_to_base.setOrigin( tf::Vector3(0, 0, GH) );
    tf::Quaternion q_m;
    q_m.setRPY(0 , 0, 0);  // translate the camera link frame into pitch frame

    tf_yaw_to_base.setRotation(q_m);

#ifdef DEBUG_TRANS
    br_yaw_to_base.sendTransform(tf::StampedTransform(tf_yaw_to_base, msg->stamp, "/map", cameraYawFrameID_));
#else
    br_yaw_to_base.sendTransform(tf::StampedTransform(tf_yaw_to_base, msg->stamp, "/base_link", cameraYawFrameID_));
#endif
}

int main(int argc, char** argv){
    ros::init(argc, argv, "broadcast_camera_frame");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

		// the first string is the variable name used by roslaunch, followed by variable value defined by roslaunch, and the default value
		pnh.param("camera_link_frame_id", cameraLinkFrameID_, cameraLinkFrameID_);
		pnh.param("camera_pitch_frame_id", cameraPitchFrameID_, cameraPitchFrameID_);
		pnh.param("camera_yaw_frame_id", cameraYawFrameID_, cameraYawFrameID_);
		pnh.param("point_cloud_source", pointCloudSourceTopic_, pointCloudSourceTopic_);
		pnh.param("camera_pitch_offset", OFFSET, OFFSET);
		pnh.param("ground_to_base_height", GH, GH);

		ros::Subscriber sub_acc = nh.subscribe("/camera_pitch", 3, pitchCallback);
		ros::Subscriber sub_yaw = nh.subscribe<std_msgs::UInt16MultiArray> ("servo", 1, yawCallback);
		ros::Subscriber sub = nh.subscribe<std_msgs::Header>("pointcloud/header", 3, tfCallback);
//		ros::Subscriber sub = nh.subscribe<std_msgs::Empty>("test", 3, tfCallback);

    while (ros::ok())
        {
            if (ros::param::has("camera_pitch_offset"))
                ros::param::get("camera_pitch_offset", OFFSET);

            if (ros::param::has("ground_to_base_height"))
                ros::param::get("ground_to_base_height", GH);

            ros::spinOnce();
        }
    return 0;
};
