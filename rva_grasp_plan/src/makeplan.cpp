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
    std::cerr << minPt.z << std::endl;
}

void MakePlan::smartOffset(pcl::PointXYZRGB &avrPt)
{

}

void MakePlan::removeNans(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.5, 1.0);
    pass.filter(*cloud_out);
}

bool MakePlan::process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, float a, float b, float c, float d, pcl::PointXYZRGB &avrPt)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ro (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_pt (new pcl::PointCloud<pcl::PointXYZRGB>);
    removeOutliers(cloud_in, cloud_filtered_ro);
    removeNans(cloud_filtered_ro, cloud_filtered_pt);
    getAveragePoint(cloud_filtered_pt, avrPt);

    return true;
}
