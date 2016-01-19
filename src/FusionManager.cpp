#include "FusionManager.h"

#include <vector>

#include <boost/chrono/thread_clock.hpp>

using namespace boost::chrono;

namespace MobileFusion {
    FusionManager::FusionManager()
    :
    renderer_(new CloudCompareRenderer())
    ,voxelsize_(5.f)
    ,tsdf_(new cpu_tsdf::TSDFVolumeOctree) {
         mat_ = Eigen::Matrix4f::Identity();
         tsdf_->setGridSize(1., 1., 1.);
         tsdf_->setResolution(128, 128, 128);
         tsdf_->setIntegrateColor(true);
         tsdf_->reset();
    }

    FusionManager::~FusionManager() {
    }

    void FusionManager::onFrame(cv::Mat &rgb, cv::Mat &depth) {
        std::cout<<"FusionManager_onFrame"<<std::endl;
    }

    void FusionManager::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        clouds_.push_back(cloud);
        if(clouds_.size()>=2) {
            Eigen::Matrix4d mat_4d(mat_.cast<double>());
            Eigen::Affine3d affine(mat_4d);
            // tsdf->integrateCloud(*clouds_[i],empty_cloud_,affine);
            mat_ *= getIcpTransformation(clouds_[clouds_.size() - 1], clouds_[clouds_.size() - 2]);
        }
    }


    Eigen::Matrix4f FusionManager::getIcpTransformation(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_source,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_target) {

        //thread_clock::time_point start = thread_clock::now();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sourceDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_targetDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);

        //make dense
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_source, *cloud_source, indices);
        pcl::removeNaNFromPointCloud(*cloud_target, *cloud_target, indices);


        //downsample clouds
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setInputCloud(cloud_source);
        filter.setLeafSize(voxelsize_, voxelsize_, voxelsize_);
        filter.filter(*cloud_sourceDownsampled);

        filter.setInputCloud(cloud_target);
        filter.setLeafSize(voxelsize_, voxelsize_, voxelsize_);
        filter.filter(*cloud_targetDownsampled);
        //thread_clock::time_point start2 = thread_clock::now();

        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
        gicp.setInputSource(cloud_sourceDownsampled);
        gicp.setInputTarget(cloud_targetDownsampled);

        //thread_clock::time_point start3 = thread_clock::now();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_registered(new pcl::PointCloud<pcl::PointXYZRGB>);
        gicp.align(*cloud_source_registered);

        //thread_clock::time_point start4 = thread_clock::now();

        if(!gicp.hasConverged())
            std::cout << "Gicp does not converged! you need to reset parameters of gicp!" << std::endl;
        //thread_clock::time_point stop = thread_clock::now();

        renderer_->onCloudFrame(cloud_targetDownsampled, cloud_source_registered);

        //std::cout<<"duration1: "<<duration_cast<nanoseconds>(start4-start3).count() <<std::endl;
        //std::cout<<"duration2: "<<duration_cast<nanoseconds>(start3-start2).count() <<std::endl;
        //std::cout<<"duration3: "<<duration_cast<nanoseconds>(start2-start).count() <<std::endl;
        //std::cout<<"duration4: "<<duration_cast<nanoseconds>(stop-start).count() <<std::endl;

        return gicp.getFinalTransformation();
    }

}


