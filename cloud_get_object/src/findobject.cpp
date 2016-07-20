#include "findobject.h"

using namespace std;

//globe parameter

double axisLength_ = 0.1;
double br_ = 0.2;
double bg_ = 0.4;
double bb_ = 0.4;

//normal threshold
double th_norm_ = 0.7;
double th_smooth_ = 6;

//length threshold
double th_leaf_ = 0.01;
double th_deltaz_ = th_leaf_;

//area threshold
double th_area_ = 0.01;

float fx_ = 525.757;
float fy_ = 527.5;
float cx_ = 303.814;
float cy_ = 233.974;

int th_size_g_ = 2500;// pointcloud min size threshold for region growing, case: ground
int th_size_p_ = 250; // case: table

pcl::ModelCoefficients::Ptr coeff_ (new pcl::ModelCoefficients) ;

findobject::findobject()
{
}

void findobject::getParameters(float ground_height, bool istabletop)
{
    isTableTop_ = istabletop;
}

void findobject::findObjectInCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_in, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_in, std::vector<float> coeff_msg)
{
    cerr << endl;
    cerr << "-------- Get object processing loop started --------" << endl;
    if (coeff_msg.size() != 4)
        {
            cerr << "No plane coeff, return." << endl;
            return;
        }

    calPlaneCoeff(plane_in, coeff_msg);

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fit_src (new pcl::PointCloud<pcl::PointXYZRGB>());
//    preProcess(src_in, fit_src);

    // get cloud upon plane hull
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ex_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    extractInliers(src_in, plane_in, coeff_, ex_cloud);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fit_upon (new pcl::PointCloud<pcl::PointXYZRGB>());
    downSampling(ex_cloud, fit_upon);
    cloud_upon_hull = fit_upon;

//    calColorRegionGrowing(ex_cloud);
    calRegionGrowing(ex_cloud);    //dont use downsampling cloud to compute region grow!
}

void findobject::calColorRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud_in);
//    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (0.035);
    reg.setPointColorThreshold (12);
    reg.setRegionColorThreshold (12);

    if (isTableTop_)
        reg.setMinClusterSize (th_size_p_);
    else
        reg.setMinClusterSize (th_size_g_);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    processEachInliers(clusters, cloud_in);
}

void findobject::calRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
		ne.setInputCloud (cloud);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZRGB> ());
		ne.setSearchMethod (tree_n);
		ne.setRadiusSearch (0.02);//in mm

		// Compute the features
		pcl::PointCloud<pcl::Normal >::Ptr cloud_nor(new pcl::PointCloud<pcl::Normal >());
		ne.compute (*cloud_nor);

		pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
		pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

    if (isTableTop_)
        reg.setMinClusterSize (th_size_p_);
    else
        reg.setMinClusterSize (th_size_g_);

		reg.setMaxClusterSize (307200);
		reg.setSearchMethod (tree);
		reg.setNumberOfNeighbours (20);
		reg.setInputCloud (cloud);
		//reg.setIndices (indices);
		reg.setInputNormals (cloud_nor);
		reg.setSmoothnessThreshold (th_smooth_ / 180.0 * M_PI);
		//reg.setCurvatureThreshold (0);

		std::vector <pcl::PointIndices> clusters;
		reg.extract (clusters);

		// region grow tire the whole cloud apart, process each part to see if they come from the same plane
		processEachInliers(clusters, cloud);
}

bool findobject::checkInAir(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_fit_part, int &added_point_idx)
{
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D (*cloud_fit_part, minPt, maxPt);

    // val is the lowest point of the 'in air cloud' distance to the plane
    float val = minPt.x * coeff_->values[0] + minPt.y * coeff_->values[1] + minPt.z * coeff_->values[2] + coeff_->values[3];

    if (val > 2 * th_deltaz_ )
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB> ());
            projectCloud(coeff_,cloud_fit_part,cloud_p);

            pcl::PointXYZRGB pTemp;
            float ztemp = cloud_p->points[0].z;

            // get the point which has the minium z value ( which is the nearest point to the camera ) and get its 2D index in image
            for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator pit = cloud_p->begin();pit != cloud_p->end();++pit)
                {
                    if (pit->z < ztemp)
                        {
                            ztemp = pit->z;
                            pTemp.z = ztemp;
                            pTemp.x = pit->x;
                            pTemp.y = pit->y;
                        }
                }

            float x_img = pTemp.x * fx_ / pTemp.z + cx_;
            float y_img = pTemp.y * fy_ / pTemp.z + cy_;
            added_point_idx = (int)y_img * 640 + (int)x_img;

            if (added_point_idx < 640 * 480)
                return true;
            else
                return false;
        }
    else
        {
            return false;
        }
}

void findobject::processEachInliers(std::vector<pcl::PointIndices> indices_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
    if (indices_in.empty())
        {
            cerr << "processEachInliers: Region growing didn't get cloud cluster.\n" << endl;
        }
    else
        {
            unsigned int k = 0;
            for (std::vector<pcl::PointIndices>::const_iterator it = indices_in.begin (); it != indices_in.end (); ++it)
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fit_part (new pcl::PointCloud<pcl::PointXYZRGB> ());
                    pcl::PointIndices object_id;

                    int count = 0;
                    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                        {
                            int idx = *pit;
                            cloud_fit_part->points.push_back (cloud_in->points[idx]);
                            count++;

                            object_id.indices.push_back(upon_hull_id.indices[idx]);
                        }
                    cerr << "processEachInliers: Number of points in cluster " << k << " :" << count << endl;

                    int bottom_id = 0;
                    if (checkInAir(cloud_fit_part, bottom_id))
                        {
                            cerr << "Cluster " << k << " is in air." << endl;
                            object_id.indices.push_back(bottom_id);
                        }

                    // reassemble the cloud part
                    cloud_fit_part->width = cloud_fit_part->points.size ();
                    cloud_fit_part->height = 1;
                    cloud_fit_part->is_dense = true;

                    objects.push_back(cloud_fit_part);
                    objects_cloud_id.push_back(object_id);//store current pointcloud indices (inherit from source cloud)

                    k++;
                }
        }
}

void findobject::downSampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{
    cerr << "PointCloud before filtering: " << cloud_in->width * cloud_in->height
              << " data points (" << pcl::getFieldsList (*cloud_in) << ")." << endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (0.005, 0.005, 0.005);
    vg.filter (*cloud_out);

    cerr << "PointCloud after filtering: " << cloud_out->width * cloud_out->height << endl;
}

void findobject::extractInliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud,
                                pcl::ModelCoefficients::Ptr coeff, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
    pcl::PointIndices::Ptr upper_id (new pcl::PointIndices);
    getCloudUpper(src_cloud, coeff, upper_id);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr upper_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    getCloudByInliers(src_cloud, upper_cloud, upper_id, false, false);

    pcl::PointIndices::Ptr inhull_id (new pcl::PointIndices);
    getCloudInHull(upper_cloud, coeff, plane_cloud, inhull_id);

    getCloudByInliers(upper_cloud,cloud_out, inhull_id, false, false);

    getParentIndices(upper_id, inhull_id, upon_hull_id);
}

void findobject::getParentIndices(pcl::PointIndices::Ptr p_id, pcl::PointIndices::Ptr c_id, pcl::PointIndices &out_id)
{
    for (size_t i = 0; i < c_id->indices.size(); i++)
        {
            int idx = c_id->indices[i];
            out_id.indices.push_back(p_id->indices[idx]);
        }
}

void findobject::getCloudInHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr upper_cloud, pcl::ModelCoefficients::Ptr coeff,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud, pcl::PointIndices::Ptr &inhull)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr upper_cloud_2d (new pcl::PointCloud<pcl::PointXYZRGB>());
    projectCloud(coeff, upper_cloud, upper_cloud_2d);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud_c (new pcl::PointCloud<pcl::PointXYZRGB> ());
    plane_cloud_c->resize(plane_cloud->size());

    std::vector<pcl::Vertices> vertices;
    pcl::Vertices vt;

    size_t ct = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = plane_cloud->begin();pit != plane_cloud->end();++pit)
        {
            plane_cloud_c->points[ct].x = pit->x;
            plane_cloud_c->points[ct].y = pit->y;
            plane_cloud_c->points[ct].z = pit->z;
            plane_cloud_c->points[ct].r = 1;
            plane_cloud_c->points[ct].g = 1;
            plane_cloud_c->points[ct].b = 1;
            vt.vertices.push_back(ct);
            ct++;
        }

    vertices.push_back(vt);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::CropHull<pcl::PointXYZRGB> cropHull;
    cropHull.setHullIndices(vertices);
    cropHull.setHullCloud(plane_cloud_c);
    cropHull.setDim(2);
    cropHull.setCropOutside(true);
    cropHull.setInputCloud(upper_cloud_2d);

    cropHull.filter(inhull->indices);
}

void findobject::getCloudUpper(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::ModelCoefficients::Ptr coeff, pcl::PointIndices::Ptr &upper)
{
//    float jud = -10*coeff->values[1] - 10 *coeff->values[2] + coeff->values[3];
    size_t ct = 0;
    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator pit = src_cloud->begin();pit != src_cloud->end();++pit)
        {
            float val = pit->x * coeff->values[0] + pit->y * coeff->values[1] + pit->z * coeff->values[2] + coeff->values[3];

            //positive value means the point is uppon table
            if (val > th_deltaz_)
                {
                    upper->indices.push_back(ct);
                }
            ct++;
        }
}

void findobject::getCloudByInliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out,
                       pcl::PointIndices::Ptr inliers, bool negative, bool organized)
{
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setNegative (negative);
    extract.setInputCloud (cloud_in);
    extract.setIndices (inliers);
    extract.setKeepOrganized(organized);
    extract.filter (*cloud_out);
}

void findobject::calPlaneCoeff(pcl::PointCloud< pcl::PointXYZ >::Ptr plane_in, std::vector<float> coeff_msg)
{
    coeff_->values.clear();
    coeff_->values.push_back(coeff_msg[0]);
    coeff_->values.push_back(coeff_msg[1]);
    coeff_->values.push_back(coeff_msg[2]);
    coeff_->values.push_back(coeff_msg[3]);
    cerr << "Plane coeffs are: " << coeff_msg[0] << " " << coeff_msg[1] << " " << coeff_msg[2] << " "  << coeff_msg[3] << " ."<< endl;
}

void findobject::projectCloud(pcl::ModelCoefficients::Ptr coeff_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_in);
    proj.setModelCoefficients(coeff_in);
    proj.filter(*cloud_out);
}
