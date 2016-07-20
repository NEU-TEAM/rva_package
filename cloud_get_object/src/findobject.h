#ifndef FINDOBJECT_H
#define FINDOBJECT_H

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
#include <pcl/filters/crop_hull.h>
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
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/mls.h>

class findobject
{
public:
    findobject();

    void getParameters(float ground_height, bool istabletop);
    void findObjectInCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_in, pcl::PointCloud< pcl::PointXYZ >::Ptr plane_in, std::vector<float> coeff_msg);

    bool isTableTop_;
    std::vector < pcl::PointCloud< pcl::PointXYZRGB>::Ptr> objects;
    std::vector<pcl::PointIndices> objects_cloud_id;
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_upon_hull;

private:
    void downSampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);
    void extractInliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud,
                                          pcl::ModelCoefficients::Ptr coeff, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);
    void calPlaneCoeff(pcl::PointCloud< pcl::PointXYZ >::Ptr plane_in, std::vector<float> coeff_msg);
    void getCloudUpper(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::ModelCoefficients::Ptr coeff, pcl::PointIndices::Ptr &upper );
    void getCloudByInliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out,
                           pcl::PointIndices::Ptr inliers, bool negative, bool organized);

    void getCloudInHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr upper_cloud, pcl::ModelCoefficients::Ptr coeff,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud, pcl::PointIndices::Ptr &inhull);

    void projectCloud(pcl::ModelCoefficients::Ptr coeff_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);

    void calColorRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    void calRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    void processEachInliers(std::vector<pcl::PointIndices> indices_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    void getParentIndices(pcl::PointIndices::Ptr p_id, pcl::PointIndices::Ptr c_id, pcl::PointIndices &out_id);

    bool checkInAir(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_fit_part, int &added_point_idx);

    pcl::PointIndices upon_hull_id;

};

#endif // FINDOBJECT_H
