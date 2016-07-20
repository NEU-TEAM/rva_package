#ifndef UTILITIES_H
#define UTILITIES_H

//STL
#include <string>
#include <vector>
#include <math.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

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

#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/point_types_conversion.h> this function can cause error, do not use


class Utilities
{
public:
    Utilities();
    void getParameters(std::string argString, std::vector <std::string> &outString, std::vector <float> &outNum);
    static void generateName(int count, std::string pref, std::string surf, std::string &name);
    static void pointTypeTransfer(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);
    static void ecExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices,
                           double th_cluster, int minsize, int maxsize);
    static void cutCloud(pcl::ModelCoefficients::Ptr coeff_in, double th_distance, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);

    static void rotateCloudXY(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_out,
                                             float rx, float ry, Eigen::Matrix4f &transform_inv);

    static void rotateBack(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                                                  Eigen::Matrix4f transform_inv);

    static void estimateNorCurv(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal >::Ptr &cloud_out);
    static void preProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
};

#endif // UTILITIES_H
