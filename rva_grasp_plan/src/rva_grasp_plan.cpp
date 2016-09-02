#include <ros/ros.h>

#include <math.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16MultiArray.h>
#include <visualization_msgs/Marker.h>

//Custom message
//#include <rva_central_control/target.h>
#include <recognize_with_vgg/Target_Recognized_Object.h>

//PCL-ROS
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

//PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>

#include "makeplan.h"

using namespace std;


typedef recognize_with_vgg::Target_Recognized_Object TRO;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

//ros::Publisher graspPubInfo_;
ros::Publisher markerPub_;
ros::Publisher graspPubStatus_;

enum ModeType{m_wander, m_recognize, m_search, m_track};
int modeType_ = m_wander;
int modeTypeTemp_ = m_wander;

string param_running_mode = "/status/running_mode";
bool hasGraspPlan_ = false;

pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
// support plane param
float pa_;
float pb_;
float pc_;
float pd_;

// marker type
uint32_t shape = visualization_msgs::Marker::ARROW;

void getCloudByInliers(PointCloud::Ptr cloud_in, PointCloud::Ptr &cloud_out,
                       pcl::PointIndices::Ptr inliers, bool negative, bool organized)
{
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setNegative (negative);
    extract.setInputCloud (cloud_in);
    extract.setIndices (inliers);
    extract.setKeepOrganized(organized);
    extract.filter (*cloud_out);
}

void msgToCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
		cloud_out->height = cloud_in->height;
		cloud_out->width  = cloud_in->width;
		cloud_out->is_dense = false;
		cloud_out->resize(cloud_out->height * cloud_out->width);

		for (size_t i = 0; i < cloud_out->size(); ++i)
				{
						cloud_out->points[i].x = cloud_in->points[i].x;
						cloud_out->points[i].y = cloud_in->points[i].y;
						cloud_out->points[i].z = cloud_in->points[i].z;
						cloud_out->points[i].r = cloud_in->points[i].r;
						cloud_out->points[i].g = cloud_in->points[i].g;
						cloud_out->points[i].b = cloud_in->points[i].b;
				}
}

void normalize(float &pa, float &pb, float &pc, float scale)
{
		// make the vector (pa, pb, pc) length equal to scale (in m)
		float avr = pow(pa_, 2) + pow(pb_, 2) + pow(pc_, 2);
		pa = pa_ / avr * scale;
		pb = pb_ / avr * scale;
		pc = pc_ / avr * scale;
}

void resultCallback(const TRO::ConstPtr & msg)
{
		if (modeType_ != m_recognize)
				{
						return;
				}

    inliers->indices.clear();
    for (size_t i = 0; i < msg->objects_pixels.vector_pixels.size(); i++)
        {
            inliers->indices.push_back(msg->objects_pixels.vector_pixels[i]);
        }

    pa_ = msg->support_plane.x;
    pb_ = msg->support_plane.y;
    pc_ = msg->support_plane.z;
    pd_ = msg->support_plane.w;
}

void cloudCallback(const PointCloud::ConstPtr& source_msg)
{
		if (modeType_ != m_recognize)
				{
						return;
				}

    if (source_msg->empty())
        {
            ROS_ERROR("Can't get source cloud message.\n");
            return;
        }

    if (inliers->indices.empty())
        {
            ROS_ERROR_THROTTLE(5, "Object to grasp has not been found.\n");
            return;
        }

    PointCloud::Ptr cloud_in (new PointCloud());
    msgToCloud(source_msg, cloud_in);

    PointCloud::Ptr cloud_out (new PointCloud ());
    getCloudByInliers(cloud_in, cloud_out, inliers, false, false);

    MakePlan MP;
    pcl::PointXYZRGB avrPt;
    hasGraspPlan_ = MP.process(cloud_out, pa_, pb_, pc_, pd_, avrPt);

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = source_msg->header.frame_id;
    marker.header.stamp = pcl_conversions::fromPCL(source_msg->header.stamp);

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "grasp";
    marker.id = 0;

    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.points.resize(2);
    marker.points[0].x = avrPt.x;
    marker.points[0].y = avrPt.y;
    marker.points[0].z = avrPt.z;

    float la, lb, lc;
    normalize(la, lb, lc, 0.15);
    marker.points[1].x = avrPt.x + la;
    marker.points[1].y = avrPt.y + lb;
    marker.points[1].z = avrPt.z + lc;

    // The point at index 0 is assumed to be the start point, and the point at index 1 is assumed to be the end.

    // scale.x is the shaft diameter, and scale.y is the head diameter. If scale.z is not zero, it specifies the head length.
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.01;
    marker.scale.y = 0.015;
    marker.scale.z = 0.04;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    markerPub_.publish(marker);
}

int main(int argc, char **argv)
{
		ros::init(argc, argv, "rva_grasp_plan");

		ros::NodeHandle nh;

//		graspPubInfo_ = nh.advertise<std_msgs::UInt16MultiArray>("servo", 1);
		markerPub_ = nh.advertise<visualization_msgs::Marker>("grasp/marker", 1);

		graspPubStatus_ = nh.advertise<std_msgs::Bool>("status/grasp/feedback", 1);

		ros::Subscriber sub_res = nh.subscribe<TRO>("recognize/confirm/result", 1, resultCallback);
		ros::Subscriber sub_clouds = nh.subscribe("/point_cloud", 1, cloudCallback);

		ROS_WARN("Grasping plan function initialized!\n");

		while (ros::ok())
				{
						if (ros::param::has(param_running_mode))
								{
										ros::param::get(param_running_mode, modeTypeTemp_);
										modeType_ = modeTypeTemp_;
								}

						std_msgs::Bool flag;
						flag.data = true;

						ros::spinOnce();

						flag.data = hasGraspPlan_;
						graspPubStatus_.publish(flag);
				}

		return 0;
}
