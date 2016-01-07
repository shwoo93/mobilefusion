#include "util3d_registration.h"

#include <pcl/registration/gicp.h>
#include <pcl/registration/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>


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
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_source,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_target) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sourceDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_targetDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);


        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_source, *cloud_source, indices);
        pcl::removeNaNFromPointCloud(*cloud_target, *cloud_target, indices);

        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setInputCloud(cloud_source);
        filter.setLeafSize(0.01f, 0.01f, 0.01f);
        filter.filter(*cloud_sourceDownsampled);

        filter.setInputCloud(cloud_target);
        filter.setLeafSize(0.01f, 0.01f, 0.01f);
        filter.filter(*cloud_targetDownsampled);


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
