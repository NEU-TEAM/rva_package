#ifndef FINDPLANE_H
#define FINDPLANE_H

//STL
#include <iostream>
#include <math.h>
#include <vector>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_cloud.h>

#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/mls.h>

//This
#include "utilities.h"

typedef pcl::PointXYZRGBNormal PointT;

class FindPlane
{
public:
    FindPlane();
    ~FindPlane();

    void getParameters(float ground_height);
    void findPlaneInCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_in);

    std::vector < pcl::ModelCoefficients::Ptr > plane_coeff;
    std::vector < pcl::ModelCoefficients::Ptr > ground_coeff;
    std::vector < pcl::PointCloud< pcl::PointXYZ >::Ptr> plane_hull;
    std::vector < pcl::PointCloud< pcl::PointXYZ >::Ptr> ground_hull;

private:
    void preProcess(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_out);
    void extractPlane(double z_in, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_fit_plane, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_mono);
    void mergeCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_fit_plane, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_mono);
    void getAverage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, double &avr, double &deltaz);

    void projectCloud(pcl::ModelCoefficients::Ptr coeff_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);

    void getCloudByInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                           pcl::PointIndices::Ptr inliers, bool negative, bool organized);

    double filtCloudByZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
    void processEachInliers(std::vector<pcl::PointIndices> indices_in, pcl::PointCloud<PointT>::Ptr cloud_in);
    void getCloudByConditions(pcl::PointCloud<PointT>::Ptr cloud_source, pcl::PointIndices::Ptr &inliers_plane, double thn);
    void getCloudByInliers(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr &cloud_out,
                           pcl::PointIndices::Ptr inliers, bool negative, bool organized);
    void calRegionGrowing(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

    std::vector < double > planeZVector_;
    std::vector < double > planeCollectVector_;

    float gh_;

};

#endif // FINDPLANE_H
