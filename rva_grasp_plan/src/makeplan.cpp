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

bool MakePlan::getAveragePoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointXYZRGB &avrPt)
{
    if (!cloud_in->points.size())
        {
            return false;
        }
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*cloud_in,minPt, maxPt);
    avrPt.x = (maxPt.x+ minPt.x) / 2;
    avrPt.y = (maxPt.y + minPt.y) / 2;
    avrPt.z = (maxPt.z + minPt.z) / 2;
    if (avrPt.z < 0.5)
        {
            return false;
        }
    else
        {
            return true;
        }
}

void MakePlan::smartOffset(pcl::PointXYZRGB &avrPt)
{

}

void MakePlan::removeField(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
    pcl::PointIndices::Ptr indices_x(new pcl::PointIndices);
    pcl::PointIndices::Ptr indices_xy(new pcl::PointIndices);

    pcl::PassThrough<pcl::PointXYZRGB> ptfilter; // Initializing with true will allow us to extract the removed indices
    ptfilter.setInputCloud (cloud_in);
    ptfilter.setFilterFieldName ("x");
    ptfilter.setFilterLimits (-2.0, 2.0);
    ptfilter.filter (indices_x->indices);

    ptfilter.setIndices (indices_x);
    ptfilter.setFilterFieldName ("y");
    ptfilter.setFilterLimits (-2.0, 2.0);
    ptfilter.filter (indices_xy->indices);

    ptfilter.setIndices (indices_xy);
    ptfilter.setFilterFieldName ("z");
    ptfilter.setFilterLimits (0.5,2.0);
    ptfilter.filter (*cloud_out);
}

void MakePlan::removeNans(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
      std::vector< int >	index;
      pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, index);
}

bool MakePlan::process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, float a, float b, float c, float d, pcl::PointXYZRGB &avrPt)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_out (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_nan (new pcl::PointCloud<pcl::PointXYZRGB>);
    removeOutliers(cloud_in, cloud_filtered_out);
    removeField(cloud_filtered_out, cloud_filtered_nan);
    bool success = getAveragePoint(cloud_filtered_nan, avrPt);
    return success;
}
