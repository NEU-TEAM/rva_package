#include "getsourcecloud.h"

GetSourceCloud::GetSourceCloud()
{
}

template<class T>
inline bool uIsFinite(const T & value)
{
#if _MSC_VER
	return _finite(value) != 0;
#else
	return std::isfinite(value);
#endif
}


float getDepth(
		const cv::Mat & depthImage,
		float x, float y,
		bool smoothing,
		float maxZError,
		bool estWithNeighborsIfNull)
{

	int u = int(x+0.5f);
	int v = int(y+0.5f);
	if(u == depthImage.cols && x<float(depthImage.cols))
		{
			u = depthImage.cols - 1;
		}
	if(v == depthImage.rows && y<float(depthImage.rows))
		{
			v = depthImage.rows - 1;
		}

	if(!(u >=0 && u<depthImage.cols && v >=0 && v<depthImage.rows))
		{
			return 0;
		}

	bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

	// Inspired from RGBDFrame::getGaussianMixtureDistribution() method from
	// https://github.com/ccny-ros-pkg/rgbdtools/blob/master/src/rgbd_frame.cpp
	// Window weights:
	//  | 1 | 2 | 1 |
	//  | 2 | 4 | 2 |
	//  | 1 | 2 | 1 |
	int u_start = std::max(u-1, 0);
	int v_start = std::max(v-1, 0);
	int u_end = std::min(u+1, depthImage.cols-1);
	int v_end = std::min(v+1, depthImage.rows-1);

	float depth = 0.0f;
	if(isInMM)
		{
			if(depthImage.at<unsigned short>(v,u) > 0 &&
				 depthImage.at<unsigned short>(v,u) < std::numeric_limits<unsigned short>::max())
				{
					depth = float(depthImage.at<unsigned short>(v,u))*0.001f;
				}
		}
	else
		{
			depth = depthImage.at<float>(v,u);
		}

	if((depth==0.0f || !uIsFinite(depth)) && estWithNeighborsIfNull)
		{
			// all cells no2 must be under the zError to be accepted
			float tmp = 0.0f;
			int count = 0;
			for(int uu = u_start; uu <= u_end; ++uu)
				{
					for(int vv = v_start; vv <= v_end; ++vv)
						{
							if((uu == u && vv!=v) || (uu != u && vv==v))
								{
									float d = 0.0f;
									if(isInMM)
										{
											if(depthImage.at<unsigned short>(vv,uu) > 0 &&
												 depthImage.at<unsigned short>(vv,uu) < std::numeric_limits<unsigned short>::max())
												{
													depth = float(depthImage.at<unsigned short>(vv,uu))*0.001f;
												}
										}
									else
										{
											d = depthImage.at<float>(vv,uu);
										}
									if(d!=0.0f && uIsFinite(d))
										{
											if(tmp == 0.0f)
												{
													tmp = d;
													++count;
												}
											else if(fabs(d - tmp) < maxZError)
												{
													tmp+=d;
													++count;
												}
										}
								}
						}
				}
			if(count > 1)
				{
					depth = tmp/float(count);
				}
		}

	if(depth!=0.0f && uIsFinite(depth))
		{
			if(smoothing)
				{
					float sumWeights = 0.0f;
					float sumDepths = 0.0f;
					for(int uu = u_start; uu <= u_end; ++uu)
						{
							for(int vv = v_start; vv <= v_end; ++vv)
								{
									if(!(uu == u && vv == v))
										{
											float d = 0.0f;
											if(isInMM)
												{
													if(depthImage.at<unsigned short>(vv,uu) > 0 &&
														 depthImage.at<unsigned short>(vv,uu) < std::numeric_limits<unsigned short>::max())
														{
															depth = float(depthImage.at<unsigned short>(vv,uu))*0.001f;
														}
												}
											else
												{
													d = depthImage.at<float>(vv,uu);
												}

											// ignore if not valid or depth difference is too high
											if(d != 0.0f && uIsFinite(d) && fabs(d - depth) < maxZError)
												{
													if(uu == u || vv == v)
														{
															sumWeights+=2.0f;
															d*=2.0f;
														}
													else
														{
															sumWeights+=1.0f;
														}
													sumDepths += d;
												}
										}
								}
						}
					// set window weight to center point
					depth *= 4.0f;
					sumWeights += 4.0f;

					// mean
					depth = (depth+sumDepths)/sumWeights;
				}
		}
	else
		{
			depth = 0;
		}
	return depth;
}


pcl::PointXYZ projectDepthTo3D(
		const cv::Mat & depthImage,
		float x, float y,
		float cx, float cy,
		float fx, float fy,
		bool smoothing,
		float maxZError)
{

	pcl::PointXYZ pt;

	float depth = getDepth(depthImage, x, y, smoothing, maxZError, true);
	if(depth > 0.0f)
		{
			// Use correct principal point from calibration
			cx = cx > 0.0f ? cx : float(depthImage.cols/2) - 0.5f; //cameraInfo.K.at(2)
			cy = cy > 0.0f ? cy : float(depthImage.rows/2) - 0.5f; //cameraInfo.K.at(5)

			// Fill in XYZ
			pt.x = (x - cx) * depth / fx;
			pt.y = (y - cy) * depth / fy;
			pt.z = depth;
		}
	else
		{
			pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
		}
	return pt;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDepthRGB(const cv::Mat & imageRgb,const cv::Mat & imageDepth,
	float cx, float cy,float fx, float fy,int decimation,float maxDepth,float minDepth)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(decimation < 1)
		{
			return cloud;
		}

	bool mono;
	if(imageRgb.channels() == 3) // BGR
		{
			mono = false;
		}
	else if(imageRgb.channels() == 1) // Mono
		{
			mono = true;
		}
	else
		{
			return cloud;
		}

	//cloud.header = cameraInfo.header;
	cloud->height = imageRgb.rows/decimation;
	cloud->width  = imageRgb.cols/decimation;
	cloud->is_dense = false;
	cloud->resize(cloud->height * cloud->width);

	float rgbToDepthFactorX = 1.0f/float((imageRgb.cols / imageDepth.cols));
	float rgbToDepthFactorY = 1.0f/float((imageRgb.rows / imageDepth.rows));
	float depthFx = fx * rgbToDepthFactorX;
	float depthFy = fy * rgbToDepthFactorY;
	float depthCx = cx * rgbToDepthFactorX;
	float depthCy = cy * rgbToDepthFactorY;

	int oi = 0;
	for(int h = 0; h < imageRgb.rows && h/decimation < (int)cloud->height; h+=decimation)
		{
			for(int w = 0; w < imageRgb.cols && w/decimation < (int)cloud->width; w+=decimation)
				{
					pcl::PointXYZRGB & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));
					if(!mono)
						{
							pt.b = imageRgb.at<cv::Vec3b>(h,w)[0];
							pt.g = imageRgb.at<cv::Vec3b>(h,w)[1];
							pt.r = imageRgb.at<cv::Vec3b>(h,w)[2];
						}
					else
						{
							unsigned char v = imageRgb.at<unsigned char>(h,w);
							pt.b = v;
							pt.g = v;
							pt.r = v;
						}

					pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w*rgbToDepthFactorX, h*rgbToDepthFactorY, depthCx, depthCy, depthFx, depthFy, false,3.0);
					if(pcl::isFinite(ptXYZ) && ptXYZ.z>=minDepth && (maxDepth<=0.0f || ptXYZ.z <= maxDepth))
						{
							pt.x = ptXYZ.x;
							pt.y = ptXYZ.y;
							pt.z = ptXYZ.z;
							++oi;
						}
					else
						{
							pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
						}
				}
		}
	if(oi == 0)
		{
			PCL_WARN("Cloud with only NaN values created!\n");
		}
	return cloud;
}



void estimateNorCurv(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal >::Ptr &cloud_out,float searchR)
{
	cloud_out->height = cloud_in->height;
	cloud_out->width  = cloud_in->width;
	cloud_out->is_dense = false;
	cloud_out->resize(cloud_out->height * cloud_out->width);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mono (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_mono->height = cloud_in->height;
	cloud_mono->width  = cloud_in->width;
	cloud_mono->is_dense = false;
	cloud_mono->resize(cloud_mono->height * cloud_mono->width);

//	for (size_t i = 0; i < cloud_in->size(); ++i)
//		{
//			cloud_mono->points[i].x = cloud_in->points[i].x;
//			cloud_mono->points[i].y = cloud_in->points[i].y;
//			cloud_mono->points[i].z = cloud_in->points[i].z;
//		}

//	// Create the normal estimation class, and pass the input dataset to it
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//	ne.setInputCloud (cloud_mono);

//	// Create an empty kdtree representation, and pass it to the normal estimation object.
//	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//	ne.setSearchMethod (tree);
//	ne.setRadiusSearch (searchR);//in m

//	// Compute the features
//	pcl::PointCloud<pcl::Normal >::Ptr cloud_nor(new pcl::PointCloud<pcl::Normal >());
//	ne.compute (*cloud_nor);

//	// Setup the principal curvatures computation
//	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

//	// Provide the original point cloud (without normals)
//	principal_curvatures_estimation.setInputCloud (cloud_mono);

//	// Provide the point cloud with normals
//	principal_curvatures_estimation.setInputNormals (cloud_nor);

//	// Use the same KdTree from the normal estimation
//	principal_curvatures_estimation.setSearchMethod (tree);
//	principal_curvatures_estimation.setRadiusSearch (0.02);

//	// Actually compute the principal curvatures
//	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
//	principal_curvatures_estimation.compute (*principal_curvatures);

	for (size_t i = 0; i < cloud_out->size(); ++i)
		{
			cloud_out->points[i].x = cloud_in->points[i].x;
			cloud_out->points[i].y = cloud_in->points[i].y;
			cloud_out->points[i].z = cloud_in->points[i].z;
			cloud_out->points[i].r = cloud_in->points[i].r;
			cloud_out->points[i].g = cloud_in->points[i].g;
			cloud_out->points[i].b = cloud_in->points[i].b;
//			cloud_out->points[i].normal_x = cloud_nor->points[i].normal_x;
//			cloud_out->points[i].normal_y = cloud_nor->points[i].normal_y;
//			cloud_out->points[i].normal_z = cloud_nor->points[i].normal_z;
//			cloud_out->points[i].curvature = principal_curvatures->points[i].pc1;//pc1:max value of curv
		}
}

//void GetSourceCloud::getParam(double fx, double fy, double cx, double cy)
//{
//    fx_ = fx;
//    fy_ = fy;
//    cx_ = cx;
//    cy_ = cy;
//}

//bool GetSourceCloud::getCloud(cv::Mat color, cv::Mat depth, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr  &cloudNC)
//{
//    if (color.empty())
//        {
//            std::cerr << "color image is empty" << std::endl;
//            return false;
//        }
//    if (depth.empty())
//        {
//            std::cerr << "depth image is empty" << std::endl;
//            return false;
//        }
////    else
////        {
////            std::cerr << "images recevied" << std::endl;
////        }
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSource (new pcl::PointCloud<pcl::PointXYZRGB> ());
//    cloudSource = cloudFromDepthRGB(color, depth, cx_, cy_, fx_, fy_, 1, 3.5, 0.55);
////    estimateNorCurv(cloudSource, cloudNC, 0.02);
//    return true;
//}


bool GetSourceCloud::getCloud(cv::Mat color, cv::Mat depth, float fx, float fy, float cx, float cy, float maxDepth, float minDepth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr  &cloudSource)
{
    if (color.empty())
        {
            std::cerr << "color image is empty" << std::endl;
            return false;
        }
    if (depth.empty())
        {
            std::cerr << "depth image is empty" << std::endl;
            return false;
        }

    cloudSource = cloudFromDepthRGB(color, depth, cx, cy, fx, fy, 1, maxDepth, minDepth);
    return true;
}
