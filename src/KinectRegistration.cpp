#include <vector>

#include "KinectRegistration.h"

namespace MobileFusion {
    KinectRegistration::KinectRegistration()
        :voxelsize_(0.1f) {
        }

    KinectRegistration::~KinectRegistration() {
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr KinectRegistration::voxelize(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setLeafSize(voxelsize_, voxelsize_, voxelsize_);
        filter.setInputCloud(cloud);
        filter.filter(*output);
        return output;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr KinectRegistration::removeNaNFromPointCloud(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *output, indices);
        return output;
    }

    Eigen::Matrix4f KinectRegistration::getIcpTransformation(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_source,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_target) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sourceRemovedNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_targetRemovedNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sourceDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_targetDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);

        cloud_sourceRemovedNaN = removeNaNFromPointCloud(cloud_source);
        cloud_targetRemovedNaN = removeNaNFromPointCloud(cloud_target);

        cloud_sourceDownsampled = voxelize(cloud_sourceRemovedNaN);
        cloud_targetDownsampled = voxelize(cloud_targetRemovedNaN);

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
        gicp.setInputSource(cloud_sourceDownsampled);
        gicp.setInputTarget(cloud_targetDownsampled);

        pcl::PointCloud<pcl::PointXYZRGB> cloud_source_registered;
        gicp.align(cloud_source_registered);

        if(!gicp.hasConverged())
            std::cout << "Gicp does not converged! you need to reset parameters of gicp!" << std::endl;

        return gicp.getFinalTransformation();
    }

}




