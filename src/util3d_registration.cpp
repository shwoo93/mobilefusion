#include "util3d_registration.h"

#include <pcl/registration/gicp.h>
#include <pcl/registration/transforms.h>

namespace mobilefusion {
namespace util3d {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
            const Eigen::Matrix4f &transform) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud, *output, transform);
        return output;
    }

    Eigen::Matrix4f transformationIcp(
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_source,
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_target) {
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;

        gicp.setInputTarget(cloud_target);
        gicp.setInputSource(cloud_source);

        // You can set parameters of gicp if you needed

        pcl::PointCloud<pcl::PointXYZRGB> cloud_source_registered;
        gicp.align(cloud_source_registered);

        if (!gicp.hasConverged())
            std::cout << "Gicp does not converged! you need to reset parameters of gicp!" << std::endl;

        return gicp.getFinalTransformation();
    }

}
}
