#include "makeplan.h"

MakePlan::MakePlan()
{
}

void MakePlan::removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
//  std::cerr << "Cloud before filtering: " << std::endl;
//  std::cerr << *cloud_in << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_in);
  sor.setMeanK (50); // The number of neighbors to analyze for each point

  /*  the standard deviation multiplier to 1. What this means is that all points who have a distance larger than 1
   * standard deviation of the mean distance to the query point will be marked as outliers and removed.
   */
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_out);

//  std::cerr << "Cloud after filtering: " << std::endl;
//  std::cerr << *cloud_out << std::endl;
}

void MakePlan::getAveragePoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointXYZRGB &avrPt)
{
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D (*cloud_in, minPt, maxPt);
    avrPt.x = (maxPt.x + minPt.x) / 2;
    avrPt.y = (maxPt.y + minPt.y) / 2;
    avrPt.z = (maxPt.z + minPt.z) / 2;
}

void MakePlan::smartOffset(pcl::PointXYZRGB &avrPt)
{

}

bool MakePlan::process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, float a, float b, float c, float d, pcl::PointXYZRGB &avrPt)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    removeOutliers(cloud_in, cloud_filtered);

    getAveragePoint(cloud_filtered, avrPt);

    return true;
}
