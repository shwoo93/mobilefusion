#include "CloudRegister.h"

#include "CloudNormalProvider.h"

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
        pcl::removeNaNFromPointCloud (*source_cloud_, *source_cloud_, indices);
        pcl::removeNaNFromPointCloud (*target_cloud_, *target_cloud_, indices);

        pcl::VoxelGrid<pcl::PointXYZRGBNormal> filter;
        filter.setInputCloud (source_cloud_);
        filter.setLeafSize (voxelsize, voxelsize, voxelsize);
        filter.filter (*source_downsampled);

        filter.setInputCloud (target_cloud_);
        filter.setLeafSize (voxelsize, voxelsize, voxelsize);
        filter.filter (*target_downsampled);

        pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
        //icp.setMaxCorrespondenceDistance (10000);
        //icp.setMaxCorrespondenceDistance (0.2);
        icp.setMaximumIterations (50);

        icp.setInputTarget (target_downsampled);
        icp.setInputSource (source_downsampled);

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source_registered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        icp.align (*source_registered);


        cloud_target_downsampled = target_downsampled;
        cloud_source_registered = source_registered;

        if(icp.hasConverged())
            std::cout<<icp.getFitnessScore()<<std::endl;
        if(!icp.hasConverged())
            std::cout<<"Icp does not converged! you need to reset parameters of icp!" <<std::endl;

        global_transform_ *= icp.getFinalTransformation ();
    }

    void CloudRegister::pairAlign (
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt,
            Eigen::Matrix4f& final_transform,
            bool downsample) {

        //remove NaNpoint
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud (*cloud_src, *cloud_src, indices);
        pcl::removeNaNFromPointCloud (*cloud_tgt, *cloud_tgt, indices);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);

        //pcl::VoxelGrid<pcl::PointXYZRGB> grid;
        pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;

        //downsampling
        if (downsample) {
            //grid.setLeafSize (0.05, 0.05, 0.05);
            //grid.setInputCloud (cloud_src);
            //grid.filter (*src);

            //grid.setInputCloud (cloud_tgt);
            //grid.filter (*tgt);

            uniform_sampling.setRadiusSearch (0.05f);
            uniform_sampling.setInputCloud (cloud_src);
            uniform_sampling.filter(*src);

            uniform_sampling.setInputCloud (cloud_tgt);
            uniform_sampling.filter(*tgt);
        }
        else {
            src = cloud_src;
            tgt = cloud_tgt;
        }
        //concatenate point with its normal
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_src = CloudNormalProvider::computePointWithNormal(src);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_tgt = CloudNormalProvider::computePointWithNormal(tgt);

        //gicp
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> gicp;
        gicp.setTransformationEpsilon (1e-6);
        gicp.setMaxCorrespondenceDistance (0.1);

        gicp.setInputSource (points_with_normals_src);
        gicp.setInputTarget (points_with_normals_tgt);

        gicp.setMaximumIterations (30);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        gicp.align(*result);

        if(gicp.hasConverged()) {
            std::cout<< "score:" <<gicp.getFitnessScore() << std::endl;
            //source to target transformation
            final_transform = gicp.getFinalTransformation();
        }
        else {
            std::cout<< "gicp did not converged!" <<std::endl;
        }
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

