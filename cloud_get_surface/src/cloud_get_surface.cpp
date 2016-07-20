#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <tf/transform_broadcaster.h>

#include <string>

//This
#include "utilities.h"
#include "findplane.h"

//Custom message
//#include <cloud_get_surface/Point_Cloud_Vector.h>

// notice that these angles are in camera optical frame
float roll_ = 1.92; // rotation x, default 20 degree dep.
float pitch_ = 0.0; // rotation y
float gh_ = 1.2; // ground height

float rolltemp_ = 0.0;
int tol_ = 2; // the tolerance (degree) about camera shaking

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudMono;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudWN;

enum FoundType{t_null, t_table, t_ground, t_both};
int fType_ = t_null;

enum ModeType{m_wander, m_recognize, m_search, m_track};
int modeType_ = m_wander;
std::string param_running_mode = "/status/running_mode";

Eigen::Matrix4f transform_inv_ = Eigen::Matrix4f::Identity();

ros::Publisher cloudPubPlaneMax_ ;
ros::Publisher posePubPlaneMax_;
ros::Publisher cloudPubGround_ ;
ros::Publisher posePubGround_;

ros::Publisher feedbackGetSurfaceType_;// int 0 = nothing, 1 = table, 2 = ground, 3 = ground+table

std::string pointCloudMaxPlaneTopic_ = "point_cloud/plane_max";
std::string poseMaxPlaneTopic_ = "pose/plane_max";
std::string pointCloudGroundTopic_ = "point_cloud/ground";
std::string poseGroundTopic_ = "pose/ground";

void processHullVector(std::vector < PointCloudMono::Ptr> hull, std::vector < pcl::ModelCoefficients::Ptr > coeff,
                       PointCloudMono::Ptr  &maxhull, pcl::ModelCoefficients::Ptr &max_coeff)
{
    float area_temp = 0.0;
    for (size_t i = 0; i < hull.size(); i++)
        {
            PointCloudMono::Ptr temp (new  PointCloudMono());

            Utilities::rotateBack(hull[i], temp, transform_inv_);

            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D (*temp, minPt, maxPt);
            float area = maxPt.x - minPt.x + maxPt.y - minPt.y + maxPt.z - minPt.z;

            if (area > area_temp)
                {
                    maxhull = temp;
                    area_temp = area;
                    Eigen::Vector4f vec, vec_o;
                    vec[0] = coeff[i]->values[0];
                    vec[1] = coeff[i]->values[1];
                    vec[2] = coeff[i]->values[2];
                    vec[3] = coeff[i]->values[3];
                    vec_o = transform_inv_ * vec;
                    max_coeff->values.resize(4);
                    max_coeff->values[0] = vec_o[0];
                    max_coeff->values[1] = vec_o[1];
                    max_coeff->values[2] = vec_o[2];
                    max_coeff->values[3] = vec_o[3];
                }
        }
}

void cloudCallback(const PointCloud::ConstPtr& source_msg)
{
    if (source_msg->empty())
        {
            ROS_ERROR("Can't get source cloud message.\n");
            fType_ = t_null;
            return;
        }
    PointCloudWN::Ptr source_n (new  PointCloudWN());
    PointCloudWN::Ptr cloud_t (new PointCloudWN ());

    Utilities::estimateNorCurv(source_msg, source_n);
    Utilities::rotateCloudXY(source_n, cloud_t, roll_, pitch_, transform_inv_);
    // std::cout << "trans_inv: "<< transform_inv_ << std::endl;

    FindPlane FP;
    FP.getParameters(gh_);
    FP.findPlaneInCloud(cloud_t);

    PointCloudMono::Ptr plane_max (new PointCloudMono());
    PointCloudMono::Ptr ground_max (new PointCloudMono());
    pcl::ModelCoefficients::Ptr mcp (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr mcg (new pcl::ModelCoefficients);

    processHullVector(FP.plane_hull, FP.plane_coeff, plane_max, mcp);
    processHullVector(FP.ground_hull, FP.ground_coeff, ground_max, mcg);

    if (!plane_max->empty() && (cloudPubPlaneMax_.getNumSubscribers() || posePubPlaneMax_.getNumSubscribers()))
        {
            sensor_msgs::PointCloud2 plane_max_msg;
            pcl::toROSMsg(*plane_max, plane_max_msg);
            plane_max_msg.header.frame_id = source_msg->header.frame_id;
            plane_max_msg.header.stamp = pcl_conversions::fromPCL(source_msg->header.stamp);

            geometry_msgs::PoseStamped pose_plane_max_msg;
            pose_plane_max_msg.header = plane_max_msg.header;
            pose_plane_max_msg.pose.orientation.x = mcp->values[0];
            pose_plane_max_msg.pose.orientation.y = mcp->values[1];
            pose_plane_max_msg.pose.orientation.z = mcp->values[2];
            pose_plane_max_msg.pose.orientation.w = mcp->values[3];

            cloudPubPlaneMax_.publish(plane_max_msg);
            posePubPlaneMax_.publish(pose_plane_max_msg);
        }

    if (!ground_max->empty() && (cloudPubGround_.getNumSubscribers() || posePubGround_.getNumSubscribers()))
        {
            sensor_msgs::PointCloud2 ground_max_msg;
            pcl::toROSMsg(*ground_max, ground_max_msg);
            ground_max_msg.header.frame_id = source_msg->header.frame_id;
            ground_max_msg.header.stamp = pcl_conversions::fromPCL(source_msg->header.stamp);

            geometry_msgs::PoseStamped pose_ground_msg;
            pose_ground_msg.header = ground_max_msg.header;
            pose_ground_msg.pose.orientation.x = mcg->values[0];
            pose_ground_msg.pose.orientation.y = mcg->values[1];
            pose_ground_msg.pose.orientation.z = mcg->values[2];
            pose_ground_msg.pose.orientation.w = mcg->values[3];

            cloudPubGround_.publish(ground_max_msg);
            posePubGround_.publish(pose_ground_msg);
        }

    if (!plane_max->empty() && !ground_max->empty())
        {
            fType_ = t_both;
        }
    else if (plane_max->empty() && !ground_max->empty())
        {
            fType_ = t_ground;
        }
    else if (!plane_max->empty() && ground_max->empty())
        {
            fType_ = t_table;
        }
    else
        {
            fType_ = t_null;
        }
}


void rollCallback(const std_msgs::Float32ConstPtr & msg)
{
    /* here, roll is in camera optical frame, the angle obtained by acc is in degree,
    depression angle is smaller than 0, here we need to let depression angle be positive and plus 90
    */
    roll_ = - msg->data + 90;
    if (fabs(roll_ - rolltemp_) > tol_ )
        rolltemp_ = roll_;
    roll_ = rolltemp_  * 0.01745; // / 180.0 * 3.14159
}

int main(int argc, char **argv)
{
		ros::init(argc, argv, "cloud_get_surface");

		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");

		// the first string is the variable name used by roslaunch, followed by variable value defined by roslaunch, and the default value
		pnh.param("max_plane_cloud_topic", pointCloudMaxPlaneTopic_, pointCloudMaxPlaneTopic_);
		pnh.param("max_plane_pose_topic", poseMaxPlaneTopic_, poseMaxPlaneTopic_);
		pnh.param("ground_cloud_topic", pointCloudGroundTopic_, pointCloudGroundTopic_);
		pnh.param("ground_pose_topic", poseGroundTopic_, poseGroundTopic_);

		posePubPlaneMax_ = nh.advertise<geometry_msgs::PoseStamped>(poseMaxPlaneTopic_, 1);
		cloudPubPlaneMax_ = nh.advertise<sensor_msgs::PointCloud2>(pointCloudMaxPlaneTopic_, 1);
		posePubGround_ = nh.advertise<geometry_msgs::PoseStamped>(poseGroundTopic_, 1);
		cloudPubGround_ = nh.advertise<sensor_msgs::PointCloud2>(pointCloudGroundTopic_, 1);

		feedbackGetSurfaceType_ = nh.advertise<std_msgs::Int8>("status/get/surface/feedback", 1);

    ros::Subscriber sub_roll = nh.subscribe<std_msgs::Float32>("/camera_pitch", 1, rollCallback);
    ros::Subscriber sub = nh.subscribe<PointCloud>("/point_cloud", 1, cloudCallback);

    while (ros::ok())
        {
            if (ros::param::has(param_running_mode))
                ros::param::get(param_running_mode, modeType_);

            std_msgs::Int8 typeNum;

            if (modeType_ == m_track)
                {
                    ROS_INFO_THROTTLE(19, "Get surface is not activated.\n");
                    continue;
                }

            ros::spinOnce();

            typeNum.data = fType_;
            feedbackGetSurfaceType_.publish(typeNum);
        }

    return 0;
}
