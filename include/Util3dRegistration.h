#ifndef __UTIL3D_REGISTRATION_H__
#define __UTIL3D_REGISTRATION_H__

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace MobileFusion {
	namespace util3d {
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(
	            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
	            const Eigen::Matrix4f &transform);

	    Eigen::Matrix4f transformationIcp_Xyz(
	            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source,
	            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target);
	    Eigen::Matrix4f transformationIcp_Xyzrgb(
	            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_source,
	            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_target);
	}
}

#endif
