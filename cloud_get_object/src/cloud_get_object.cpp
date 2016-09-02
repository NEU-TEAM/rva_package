#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <tf/transform_broadcaster.h>

//This
#include "findobject.h"

//Custom message
#include <cloud_get_surface/Point_Cloud_Vector.h>
#include <cloud_get_object/Objects_Pixels_Vector.h>
#include <cloud_get_object/Object_Image_Pixels.h>

using namespace std;

enum ModeType{m_wander, m_recognize, m_search, m_track};
int modeType_ = m_wander;
string param_running_mode = "/status/running_mode";

enum TargetType{t_null, t_onTable, t_onGround, t_onHead, t_onHand};
int tgtType_ = t_null;
string param_target_type = "/status/target/type";

int fType_ = 0;

// max plane coeffs:
float a_, b_, c_, d_;

float gh_ = 1.2; // ground height

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudMono;
typedef cloud_get_surface::Point_Cloud_Vector PCV;
typedef cloud_get_object::Object_Image_Pixels OIP;
typedef cloud_get_object::Objects_Pixels_Vector OPV;

ros::Publisher cloudPubUponHull_ ;
ros::Publisher cloudPubObjects_ ;
ros::Publisher idPubObjects_;
ros::Publisher statusPubGetObject_;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_ (new  pcl::PointCloud<pcl::PointXYZRGB>());

std::string pointCloudMaxPlaneTopic_ = "point_cloud/plane_max";
std::string poseMaxPlaneTopic_ = "pose/plane_max";
std::string pointCloudGroundTopic_ = "point_cloud/ground";
std::string poseGroundTopic_ = "pose/ground";

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

void msgToCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
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
				}
}

void cloudCallback(const PointCloud::ConstPtr& source_msg)
{
    if (source_msg->empty())
        {
            ROS_ERROR("Can't get source point cloud message.\n");
            fType_ = 0;
            return;
        }
    msgToCloud(source_msg, source_);
}

void posePCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
    if(tgtType_ != t_onTable) return;

    a_ = msg->pose.orientation.x; // just use pose to save value. the xyzw don't obey its meaning
    b_ = msg->pose.orientation.y;
    c_ = msg->pose.orientation.z;
    d_ = msg->pose.orientation.w;
}

void poseGCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
    if(tgtType_ != t_onGround) return;

    a_ = msg->pose.orientation.x; // just use pose to save value. the xyzw don't obey its meaning
    b_ = msg->pose.orientation.y;
    c_ = msg->pose.orientation.z;
    d_ = msg->pose.orientation.w;
}

void generalProcessing(const PointCloudMono::ConstPtr& msg, bool istabletop)
{
    findobject FO;
    FO.getParameters(gh_, istabletop);

    std::vector<float> coeff_msg;
    coeff_msg.push_back(a_);
    coeff_msg.push_back(b_);
    coeff_msg.push_back(c_);
    coeff_msg.push_back(d_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_in (new pcl::PointCloud<pcl::PointXYZ>());
    msgToCloud(msg, plane_in);
    FO.findObjectInCloud(source_, plane_in, coeff_msg);

    if(cloudPubUponHull_.getNumSubscribers())
        {
            sensor_msgs::PointCloud2 upon_hull;
            pcl::toROSMsg(*FO.cloud_upon_hull, upon_hull);
            upon_hull.header.frame_id = msg->header.frame_id;
            upon_hull.header.stamp = pcl_conversions::fromPCL(msg->header.stamp);

            cloudPubUponHull_.publish(upon_hull);
        }

    PCV obj_ros_cloud_vec;
    OPV obj_pixel_vec;

    if (FO.objects.size())
        {
            fType_ = 1;

            for (size_t i = 0;  i < FO.objects.size(); i++)
                {
                    sensor_msgs::PointCloud2 obj_ros_cloud;
                    pcl::toROSMsg(*FO.objects[i], obj_ros_cloud);

                    obj_ros_cloud_vec.vector_cloud.push_back(obj_ros_cloud);

                    OIP obj_in_image;
                    for (size_t j = 0; j < FO.objects_cloud_id[i].indices.size(); j++)
                        {
                            int p = FO.objects_cloud_id[i].indices[j];
                            obj_in_image.vector_pixels.push_back(p);
                        }
                    obj_pixel_vec.vector_objects.push_back(obj_in_image);
                }

            obj_pixel_vec.header.frame_id = msg->header.frame_id;
            obj_pixel_vec.header.stamp = pcl_conversions::fromPCL(msg->header.stamp);
            obj_pixel_vec.support_plane.x = a_;
            obj_pixel_vec.support_plane.y = b_;
            obj_pixel_vec.support_plane.z = c_;
            obj_pixel_vec.support_plane.w = d_;

            obj_ros_cloud_vec.header = obj_pixel_vec.header;

            idPubObjects_.publish(obj_pixel_vec);

            if (cloudPubObjects_.getNumSubscribers())
                {
                    // only for visualization till now
                    cloudPubObjects_.publish(obj_ros_cloud_vec);
                }
        }
    else
        {
            fType_  = 0;
        }
}

void planeCallback(const PointCloudMono::ConstPtr& p_msg)
{
    if(tgtType_ != t_onTable) return;

    if (p_msg->empty())
        {
            ROS_ERROR("Can't get plane cloud message.\n");
//            fType_ = 0; //here wait p_msg have value, so don't change feedback
            return;
        }
    generalProcessing(p_msg, true);
}

void groundCallback(const PointCloudMono::ConstPtr& g_msg)
{
    if(tgtType_ != t_onGround) return;

    if (g_msg->empty())
        {
            ROS_ERROR("Can't get plane cloud message.\n");
            return;
        }
    generalProcessing(g_msg, false);
}

int main(int argc, char **argv)
{
		ros::init(argc, argv, "cloud_get_object");

		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");

		// the first string is the variable name used by roslaunch, followed by variable value defined by roslaunch, and the default value
		pnh.param("max_plane_cloud_topic", pointCloudMaxPlaneTopic_, pointCloudMaxPlaneTopic_);
		pnh.param("max_plane_pose_topic", poseMaxPlaneTopic_, poseMaxPlaneTopic_);
		pnh.param("ground_cloud_topic", pointCloudGroundTopic_, pointCloudGroundTopic_);
		pnh.param("ground_pose_topic", poseGroundTopic_, poseGroundTopic_);

		cloudPubUponHull_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud/upon_hull",1);
		cloudPubObjects_ = nh.advertise<PCV>("point_cloud_vector/objects", 1);
		idPubObjects_ = nh.advertise<OPV>("get/object/objects_in_image",1);
		statusPubGetObject_ = nh.advertise<std_msgs::Int8>("status/get/object/feedback", 1);// 0=no obj 1=have obj

		// make sure that the callback functions run in order (each callback should recive all data needed which generated by other callbacks)
		ros::Subscriber sub_source = nh.subscribe<PointCloud>("/point_cloud", 1, cloudCallback);

		ros::Subscriber sub_plane_pose = nh.subscribe<geometry_msgs::PoseStamped>(poseMaxPlaneTopic_, 1, posePCallback);
		ros::Subscriber sub_plane = nh.subscribe<PointCloudMono>(pointCloudMaxPlaneTopic_, 1, planeCallback);
		ros::Subscriber sub_ground_pose = nh.subscribe<geometry_msgs::PoseStamped>(poseGroundTopic_, 1, poseGCallback);
		ros::Subscriber sub_ground = nh.subscribe<PointCloudMono>(pointCloudGroundTopic_, 1, groundCallback);

    while (ros::ok())
        {
            if (ros::param::has(param_target_type))
                ros::param::get(param_target_type, tgtType_);
            if (ros::param::has(param_running_mode))
                ros::param::get(param_running_mode, modeType_);

             std_msgs::Int8 status;

            if (modeType_ != m_recognize && modeType_ != m_search)
                {
                    ROS_INFO_THROTTLE(23, "Get object from cloud is not activated.\n");
                    continue;
                }

            ros::spinOnce();

            status.data = fType_;
            statusPubGetObject_.publish(status);
        }

    return 0;
}
