#include "Register.h"

namespace MobileFusion {

    Register::Register()
    : voxelsize_(5.f)
    , registerpossible_(false)
    , cloud1_(new pcl::PointCloud<pcl::PointXYZRGB>)
    , cloud2_(new pcl::PointCloud<pcl::PointXYZRGB>)
    , cloud_source_registered_(new pcl::PointCloud<pcl::PointXYZRGB>)
    , transformation_(Eigen::Matrix4f::Identity()) {

    }

    Register::~Register() {

    }

    bool Register::registerPossible() {
        return registerpossible_;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Register::getICPReadyCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setInputCloud(cloud);
        filter.setLeafSize(voxelsize_, voxelsize_, voxelsize_);
        filter.filter(*cloud_Downsampled);

        return cloud_Downsampled;
    }

    void Register::ICP(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target) {

        rendererInput_.clear();
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

        icp.setInputTarget (cloud_target);
        icp.setInputSource (cloud_source);

        //icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);
        //icp.setMaximumIterations(maximumIterations);

        icp.align(*cloud_source_registered_);

        if(!icp.hasConverged())
            std::cout<<"Icp does not converged! you need to reset parameters of icp!" <<std::endl;

        rendererInput_.push_back(cloud_target);
        rendererInput_.push_back(cloud_source_registered_);

        transformation_*=icp.getFinalTransformation();
    }

    void Register::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        //just for first insertion
        if(cloud1_->empty() &&cloud2_->empty() ) {
            cloud1_ = getICPReadyCloud(cloud);
        }

        else if(!(cloud1_->empty()) && cloud2_->empty()) {
            cloud2_ = getICPReadyCloud(cloud);
            this->ICP(cloud2_, cloud1_);
            cloud1_= cloud2_;
            cloud2_->clear();
            if(registerpossible_!=true)
                registerpossible_ = true;
        }

        else {
            std::cout<<"cloud packing failed"<<std::endl;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Register::getTargetDownsampled() {
        if(rendererInput_.size()==2) {
            return rendererInput_[0];
        }
        else {
            std::cout<<"Input insufficient"<<std::endl;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Register::getSourceRegistered() {
        if(rendererInput_.size()==2) {
            return rendererInput_[1];
        }
        else {
            std::cout<<"Input insufficient"<<std::endl;
        }
    }

    Eigen::Matrix4f Register::getIcpTransformation() {
        return transformation_;
    }

    Eigen::Affine3d Register::getAffine3d(Eigen::Matrix4f T) {
        Eigen::Matrix4d T_4d(T.cast<double>());
        Eigen::Affine3d affine(T_4d);
        return affine;
    }
}
