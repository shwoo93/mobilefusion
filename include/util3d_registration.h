#ifndef UTIL3D_REGISTRATION_H_
#define UTIL3D_REGISTRATION_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace mobilefusion {
namespace util3d {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
            const Eigen::Matrix4f &transform);

    Eigen::Matrix4f transformationIcp(
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_source,
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_target);
}
}

#endif
