#include "CloudRegister.h"

namespace MobileFusion {

    CloudRegister::CloudRegister()
    : source_cloud_(new pcl::PointCloud<pcl::PointXYZRGBNormal>)
    , target_cloud_(new pcl::PointCloud<pcl::PointXYZRGBNormal>) {
        global_transform_ = Eigen::Matrix4f::Identity();
    }

    CloudRegister::~CloudRegister() {

    }

    void CloudRegister::getIcpResultCloud(
            int voxelsize,
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud_target_downsampled,
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud_source_registered) {

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source_downsampled(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target_downsampled(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*source_cloud_, *source_cloud_, indices);
        pcl::removeNaNFromPointCloud(*target_cloud_, *target_cloud_, indices);

        pcl::VoxelGrid<pcl::PointXYZRGBNormal> filter;
        filter.setInputCloud(source_cloud_);
        filter.setLeafSize(voxelsize, voxelsize, voxelsize);
        filter.filter(*source_downsampled);

        filter.setInputCloud(target_cloud_);
        filter.setLeafSize(voxelsize, voxelsize, voxelsize);
        filter.filter(*target_downsampled);

        pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
        icp.setMaximumIterations(25);

        icp.setInputTarget (target_downsampled);
        icp.setInputSource (source_downsampled);

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source_registered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        icp.align(*source_registered);


        cloud_target_downsampled = target_downsampled;
        cloud_source_registered = source_registered;

        if(icp.hasConverged())
            std::cout<<icp.getFitnessScore()<<std::endl;
        if(!icp.hasConverged())
            std::cout<<"Icp does not converged! you need to reset parameters of icp!" <<std::endl;

        global_transform_ *= icp.getFinalTransformation ();
    }

    Eigen::Affine3d CloudRegister::getCameraPose() {
        Eigen::Matrix4d CameraPose_4d (global_transform_.cast<double> ());
        Eigen::Affine3d CameraPose (CameraPose_4d);
        return CameraPose;
    }

    void CloudRegister::setSourceCloud (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
        source_cloud_ = cloud;
    }

    void CloudRegister::setTargetCloud (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
        target_cloud_ = cloud;
    }

    void CloudRegister::getSourceCloud (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud) const {
        cloud = source_cloud_;
    }

    void CloudRegister::getTargetCloud (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud) const {
        cloud = target_cloud_;
    }
}

