
#include "FusionManager.h"

#include <vector>

namespace MobileFusion {
    FusionManager::FusionManager()
    : voxelsize_(0.5f)
    , tsdf(new cpu_tsdf::TSDFVolumeOctree) {
         mat_ = Eigen::Matrix4f::Identity();
         tsdf->setGridSize(1., 1., 1.);
         tsdf->setResolution(128, 128, 128);
         tsdf->setIntegrateColor(true);
         tsdf->reset();
    }

    FusionManager::~FusionManager() {
    }

    void FusionManager::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        static int i = 0;
        clouds_.push_back(cloud);
       if(clouds_.size()>=2) {
            Eigen::Matrix4d mat_4d(mat_.cast<double>());
            Eigen::Affine3d affine(mat_4d);
            // tsdf->integrateCloud(*clouds_[i],empty_cloud_,affine);
            mat_ *= getIcpTransformation(clouds_[i+1],clouds_[i]);
            i++;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr FusionManager::voxelize(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setLeafSize(voxelsize_, voxelsize_, voxelsize_);
        filter.setInputCloud(cloud);
        filter.filter(*output);
        return output;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr FusionManager::removeNaNFromPointCloud(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *output, indices);
        return output;
    }

    Eigen::Matrix4f FusionManager::getIcpTransformation(
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

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_registered(new pcl::PointCloud<pcl::PointXYZRGB>);
        gicp.align(*cloud_source_registered);

        if(!gicp.hasConverged())
            std::cout << "Gicp does not converged! you need to reset parameters of gicp!" << std::endl;

        pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");

        viewer.addPointCloud(cloud_targetDownsampled, "target");
        viewer.addPointCloud(cloud_source_registered, "registered");

        viewer.spinOnce(1);

        return gicp.getFinalTransformation();
    }

}




