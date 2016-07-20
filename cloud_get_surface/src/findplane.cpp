#include "findplane.h"

using namespace std;

//globe parameter

double axisLength_ = 0.1;
double br_ = 0.2;
double bg_ = 0.4;
double bb_ = 0.4;

//normal threshold
double th_norm_ = 0.7;
double th_smooth_ = 8;

//length threshold
double th_leaf_ = 0.01;
double th_noise_ = th_leaf_;// the measure error of sensor
double th_ec_ = 3 * th_leaf_;
double th_deltaz_ = 2 * th_leaf_;
double th_ratio_ = 5 * th_leaf_;// flatness ratio max value of plane

//area threshold
double th_area_ = 0.1;

double globalMaxZ, globalMinZ;
double globeMid;

FindPlane::FindPlane()
{
}

FindPlane::~FindPlane()
{
}

void FindPlane::getParameters(float ground_height)
{
     gh_ = ground_height;//camera height to the ground
}

void FindPlane::preProcess(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_out)
{

//    cerr << "PointCloud before filtering: " << cloud_in->width * cloud_in->height<< " data points (" << pcl::getFieldsList (*cloud_in) << ")." << endl;

    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (th_leaf_, th_leaf_, th_leaf_);
    vg.filter (*cloud_out);

//    cerr << "PointCloud after filtering: " << cloud_out->width * cloud_out->height << endl;
}

void FindPlane::getAverage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, double &avr, double &deltaz)
{
  avr = 0;
  deltaz = 0;
  double asum = 0;
  size_t sz = cloud_in->points.size();
  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = cloud_in->begin();pit != cloud_in->end();++pit)
    {
      avr += pit->z;
    }
  avr = avr / sz;

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud_in, minPt, maxPt);
  deltaz = maxPt.z - minPt.z;
}


void FindPlane::projectCloud(pcl::ModelCoefficients::Ptr coeff_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_in);
    proj.setModelCoefficients(coeff_in);
    proj.filter(*cloud_out);
}

void FindPlane::extractPlane(double z_in, pcl::PointCloud<PointT>::Ptr  cloud_fit_plane, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_mono)
{
    pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients() );
    coeff->values.push_back(0.0);
    coeff->values.push_back(0.0);
    coeff->values.push_back(1.0);
    coeff->values.push_back(-z_in);

  if (z_in > - gh_ + 0.5)// not ground
    {
          // use original cloud as input, get projected cloud to next cluster step
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_surface (new pcl::PointCloud<pcl::PointXYZ> ());
          Utilities::cutCloud(coeff, th_deltaz_, cloud_fit_plane, cloud_projected_surface);//th_deltaz_ must 2 times bigger than th_leaf_

          std::vector<pcl::PointIndices> cluster_indices;
          Utilities::ecExtraction(cloud_projected_surface, cluster_indices, th_ratio_, 2500, 307200);

          //cout << "plane cluster number:" << cluster_indices.size() << endl;

          for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
              {
                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d_divided (new pcl::PointCloud<pcl::PointXYZ>);
                  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                      cloud_2d_divided->points.push_back (cloud_projected_surface->points[*pit]);

                  cloud_2d_divided->width = cloud_2d_divided->points.size ();
                  cloud_2d_divided->height = 1;
                  cloud_2d_divided->is_dense = true;

                  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_hull (new pcl::PointCloud< pcl::PointXYZ >);
                  pcl::ConvexHull< pcl::PointXYZ > chull;
                  chull.setInputCloud(cloud_2d_divided);
                  chull.setComputeAreaVolume(true);
                  chull.reconstruct(*cloud_hull);
                  double area_hull = chull.getTotalArea();
                  /*pcl::ConcaveHull< pcl::PointXYZ > chull;
                                chull.setAlpha(0.5);
                                chull.setInputCloud(cloud_2d_divided);
                                chull.reconstruct(*cloud_hull);*/

                  if (cloud_hull->points.size() > 3 && area_hull > th_area_)
                      {
                          cerr << "Plane area is " << area_hull << ", consider to be support surface." << endl;
                          plane_coeff.push_back(coeff);
                          plane_hull.push_back(cloud_hull);
                      }
                  else
                      {
                          cerr << "Plane area is " << area_hull << ", smaller than " << th_area_ << ", ignored." << endl;
                      }
              }
      }
  else if (z_in < - gh_)
    {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());
          projectCloud(coeff, cloud_out_mono,  cloud_projected);

          pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_hull (new pcl::PointCloud< pcl::PointXYZ >);
          pcl::ConvexHull< pcl::PointXYZ > chull;
          chull.setInputCloud(cloud_projected);
          chull.reconstruct(*cloud_hull);

          ground_coeff.push_back(coeff);
          ground_hull.push_back(cloud_hull);
    }
}


void FindPlane::getCloudByInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                       pcl::PointIndices::Ptr inliers, bool negative, bool organized)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setNegative (negative);
  extract.setInputCloud (cloud_in);
  extract.setIndices (inliers);
  extract.setKeepOrganized(organized);
  extract.filter (*cloud_out);
}

double FindPlane::filtCloudByZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
  // remove the far point from mid each loop
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local_temp (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ> ());
  cloud_temp = cloud_in;

  double mid, zrange;
  while (cloud_temp->points.size() > 2)
    {
      double dis_high = 0.0;
      double dis_low = 0.0;
      int max_high = -1;
      int max_low = -1;
      pcl::PointIndices::Ptr pointToRemove (new pcl::PointIndices);
      getAverage(cloud_temp, mid, zrange);
      if (zrange <= th_deltaz_)
        break;

      // remove both upper and bottom points
      size_t ct = 0;
      for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = cloud_temp->begin();pit != cloud_temp->end();++pit)
        {
          double dis = pit->z - mid;
          if (dis - dis_high >= th_leaf_)
            {
              dis_high = dis;
              max_high = ct;
            }
          if (dis - dis_low <= - th_leaf_)
            {
              dis_low = dis;
              max_low = ct;
            }
          ct++;
        }
      if (max_low < 0 && max_high < 0)
        break;
      if (max_high >= 0)
        pointToRemove->indices.push_back(max_high);
      if (max_low >= 0)
        pointToRemove->indices.push_back(max_low);

      getCloudByInliers(cloud_temp,cloud_local_temp,pointToRemove,true,false);
      cloud_temp = cloud_local_temp;
    }
  return mid;
}

void FindPlane::processEachInliers(std::vector<pcl::PointIndices> indices_in, pcl::PointCloud<PointT>::Ptr cloud_in)
{
  if (indices_in.empty())
    {
      cerr << "processEachInliers: Region growing get nothing.\n" << endl;
    }
  else
    {
      size_t k = 0;
      for (std::vector<pcl::PointIndices>::const_iterator it = indices_in.begin (); it != indices_in.end (); ++it)
        {
              pcl::PointCloud<PointT>::Ptr cloud_fit_part (new pcl::PointCloud<PointT> ());
              int count = 0;
              for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                      cloud_fit_part->points.push_back (cloud_in->points[*pit]);
                      count++;
                }
              // reassemble the cloud part
              cloud_fit_part->width = cloud_fit_part->points.size ();
              cloud_fit_part->height = 1;
              cloud_fit_part->is_dense = true;

              cerr << "processEachInliers: Number of points in cluster " << k << " :" << count << endl;

              //search those part which may come from same plane
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fit_part_t (new pcl::PointCloud<pcl::PointXYZ> ());
              Utilities::pointTypeTransfer(cloud_fit_part,cloud_fit_part_t);

              double z_value_of_plane_part = filtCloudByZ(cloud_fit_part_t);
              planeZVector_.push_back(z_value_of_plane_part);
              k++;
        }

      if (planeZVector_.empty())
        {
              cerr << "processEachInliers: No cloud part pass the z judgment." << endl;
              return;
        }

      sort(planeZVector_.begin(),planeZVector_.end());
    }
}


void FindPlane::getCloudByConditions(pcl::PointCloud<PointT>::Ptr cloud_source, pcl::PointIndices::Ptr &inliers_plane, double thn)
{
  for (int i = 0; i < cloud_source->points.size(); ++i)
    {
      float nz = cloud_source->points[i].normal_z;

      /// THRESHOLD
      //if point normal fulfill this criterion, consider it from plane
      if (nz > thn)
        {
          inliers_plane->indices.push_back(i);
        }
    }
}

void FindPlane::getCloudByInliers(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr &cloud_out,
                       pcl::PointIndices::Ptr inliers, bool negative, bool organized)
{
  pcl::ExtractIndices<PointT> extract;
  extract.setNegative (negative);
  extract.setInputCloud (cloud_in);
  extract.setIndices (inliers);
  extract.setKeepOrganized(organized);
  extract.filter (*cloud_out);
}


void FindPlane::calRegionGrowing(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
      pcl::RegionGrowing<PointT, pcl::Normal> reg;
      pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);

      reg.setMinClusterSize (1000);// this affect both ground and table detection, small value make detect ground easier
      reg.setMaxClusterSize (307200);
      reg.setSearchMethod (tree);
      reg.setNumberOfNeighbours (20);
      reg.setInputCloud (cloud);
      //reg.setIndices (indices);
      reg.setInputNormals (normals);
      reg.setSmoothnessThreshold (th_smooth_ / 180.0 * M_PI);
      //reg.setCurvatureThreshold (0);

      std::vector <pcl::PointIndices> clusters;
      reg.extract (clusters);

      /// region grow tire the whole cloud apart, process each part to see if they come from the same plane
      processEachInliers(clusters, cloud);
}

// merge cloud which from the same plane
void FindPlane::mergeCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_fit_plane, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_mono)
{
    cerr << "Probability plane number: " << planeZVector_.size() << endl;
    for (vector <double>::iterator cit = planeZVector_.begin(); cit != planeZVector_.end(); cit++)
    {
        extractPlane(*cit, cloud_fit_plane, cloud_out_mono);
    }
}

void FindPlane::findPlaneInCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_in)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_out (new  pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_mono (new  pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_fit_plane (new  pcl::PointCloud<pcl::PointXYZRGBNormal>());
//    preProcess(cloud_in, cloud_out);
    cloud_out = cloud_in;

    pcl::PointIndices::Ptr indices_plane_n(new pcl::PointIndices);
    getCloudByConditions(cloud_out, indices_plane_n, th_norm_);
    if (indices_plane_n->indices.empty())
        {
            cerr << "FindPlane: No cloud satisfy normal condition.\n" << endl;
            return;
        }
    getCloudByInliers(cloud_out, cloud_fit_plane, indices_plane_n, false, false);// here cloud_fit_plane get normals

    cerr << "Points may from plane: " << cloud_fit_plane->points.size() << endl;

    pcl::PointCloud<pcl::Normal>::Ptr normals_plane_curv (new pcl::PointCloud<pcl::Normal> ());
    normals_plane_curv->resize(cloud_fit_plane->size());

    for (size_t i = 0; i < cloud_fit_plane->points.size(); i++)
        {
            normals_plane_curv->points[i].normal_x = cloud_fit_plane->points[i].normal_x;
            normals_plane_curv->points[i].normal_y = cloud_fit_plane->points[i].normal_y;
            normals_plane_curv->points[i].normal_z = cloud_fit_plane->points[i].normal_z;
//            std::cerr << normals_plane_curv->points[i].normal_x << std::endl;
        }
    calRegionGrowing(cloud_fit_plane, normals_plane_curv);

    Utilities::pointTypeTransfer(cloud_out, cloud_out_mono);
    mergeCloud(cloud_fit_plane, cloud_out_mono);
}
