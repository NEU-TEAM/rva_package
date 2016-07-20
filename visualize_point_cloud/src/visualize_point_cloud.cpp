#include <ros/ros.h>

#include <std_msgs/String.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

//Custom message
#include <cloud_get_surface/Point_Cloud_Vector.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef cloud_get_surface::Point_Cloud_Vector PCV;

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (PointCloud::ConstPtr source_msg);
boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

//pcl::visualization::CloudViewer viewer("Cloud Viewer");

//void on_cloud_cb(const PointCloud::ConstPtr & source_msg)
//{
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(source_msg);

//    cloud_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "single_cloud");

//    if (!cloud_viewer->updatePointCloud (source_msg,rgb, "single_cloud"))
//    {
//        cloud_viewer->addPointCloud(source_msg,rgb,"single_cloud");

//    }
//}

//void on_object_cb(const PCV::ConstPtr & object_msg)
//{
////    PointCloud assambleCloud (new PointCloud ());
//    PointCloud::Ptr assambleCloud(new PointCloud());
//    for (size_t i = 0; i < object_msg->vector_cloud.size(); i++)
//        {
//            PointCloud pcl_object;
//            pcl::fromROSMsg(object_msg->vector_cloud[i], pcl_object);
//            for (size_t j = 0; j < pcl_object.points.size(); j++)
//                {
//                    assambleCloud->points.push_back(pcl_object.points[j]);
//                }
//        }
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(assambleCloud);

//    cloud_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "object_cloud");

//    if (!cloud_viewer->updatePointCloud (assambleCloud,rgb, "object_cloud"))
//    {
//        cloud_viewer->addPointCloud(assambleCloud,rgb,"object_cloud");
//    }
//}

void generateName(int count, std::string pref, std::string surf, std::string &name)
{
  std::ostringstream ost;
  ost << count;
  std::string temp(ost.str());
  name = pref + temp + surf;
}

void on_object_cb(const PCV::ConstPtr & object_msg)
{
    cloud_viewer->removeAllPointClouds();

    for (size_t i = 0; i < object_msg->vector_cloud.size(); i++)
        {
            PointCloud pcl_object;
            PointCloud::Ptr assambleCloud(new PointCloud());
            pcl::fromROSMsg(object_msg->vector_cloud[i], pcl_object);
            for (size_t j = 0; j < pcl_object.points.size(); j++)
                {
                    assambleCloud->points.push_back(pcl_object.points[j]);
                }

            pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> rgb(assambleCloud);
            std::string pname;
            generateName(i, "object_cloud", "", pname);
            cloud_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, pname);
            if (!cloud_viewer->updatePointCloud (assambleCloud, rgb, pname))
            {
                cloud_viewer->addPointCloud(assambleCloud,rgb,pname);
            }
        }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualize_point_cloud");

    ros::NodeHandle nh;

//    ros::Subscriber sub_clouds = nh.subscribe("/point_cloud", 1, on_cloud_cb);
    ros::Subscriber sub_clouds = nh.subscribe<PCV>("point_cloud_vector/objects", 1, on_object_cb);

    cloud_viewer->setBackgroundColor (0, 0, 0);

    cloud_viewer->initCameraParameters ();

    ros::Rate loop_rate(10);
    while (ros::ok() && !cloud_viewer->wasStopped())
    {
        cloud_viewer->spinOnce(1);
        ros::spinOnce();
        loop_rate.sleep ();
    }

    return 0;
}
