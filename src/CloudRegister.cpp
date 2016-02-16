#include "CloudRegister.h"

#include "CloudNormalProvider.h"

namespace MobileFusion {

    CloudRegister::CloudRegister()
    : target_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>) {
        global_transformation_ = Eigen::Matrix4f::Identity();
    }

    CloudRegister::~CloudRegister() {

    }

    void CloudRegister::pairAlign (
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_src,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt,
            Eigen::Matrix4f& final_transform,
            bool& hasICPConverged,
            bool downsample) {

        std::cout << "Number of cloud source : " << cloud_src->size() <<std::endl;
        std::cout << "Number of cloud target : " << cloud_tgt->size() <<std::endl;

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::UniformSampling<pcl::PointXYZRGBNormal> sampling_with_normal;
        pcl::UniformSampling<pcl::PointXYZRGB> sampling;

        if (downsample) {
            sampling_with_normal.setRadiusSearch (0.05f);
            sampling_with_normal.setInputCloud (cloud_src);
            sampling_with_normal.filter(*src);

            sampling.setRadiusSearch (0.05f);
            sampling.setInputCloud (cloud_tgt);
            sampling.filter(*tgt);
        }
        else {
            src = cloud_src;
            tgt = cloud_tgt;
        }

        std::cout << "Number of sampled cloud source : " << src->size() << std::endl;
        std::cout << "Number of sampled cloud target : " << tgt->size() << std::endl;

        //concatenate point with its normal
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_tgt = CloudNormalProvider::computePointWithNormal(tgt);

        pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
        icp.setTransformationEpsilon (1e-8);
        icp.setMaxCorrespondenceDistance (0.1);
        icp.setMaximumIterations (30);

        icp.setInputSource (src);
        icp.setInputTarget (points_with_normals_tgt);

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        icp.align(*result);

        if(icp.hasConverged()) {
            std::cout<< "score:" <<icp.getFitnessScore() << std::endl;
            final_transform = icp.getFinalTransformation().inverse();
            hasICPConverged = true;
        }
        else {
            std::cout<< "gicp did not converged!" <<std::endl;
        }
    }

    void CloudRegister::pairAlign (
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt,
            Eigen::Matrix4f& final_transform,
            bool& hasICPConverged,
            bool downsample) {

        std::cout << "Number of cloud source : " << cloud_src->size() << std::endl;
        std::cout << "Number of cloud target : " << cloud_tgt->size() << std::endl;

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

        std::cout << "Number of sampled cloud source : " << src->size() << std::endl;
        std::cout << "Number of sampled cloud target : " << tgt->size() << std::endl;

        //concatenate point with its normal
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_src = CloudNormalProvider::computePointWithNormal(src);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_tgt = CloudNormalProvider::computePointWithNormal(tgt);

        //icp
        pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
        icp.setTransformationEpsilon (1e-8);
        icp.setMaxCorrespondenceDistance (0.1);
        icp.setMaximumIterations (30);

        icp.setInputSource (points_with_normals_src);
        icp.setInputTarget (points_with_normals_tgt);

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        icp.align(*result);

        if(icp.hasConverged()) {
            std::cout<< "score:" <<icp.getFitnessScore() << std::endl;
            final_transform = icp.getFinalTransformation();
            hasICPConverged = true;
        }
        else {
            std::cout<< "gicp did not converged!" <<std::endl;
        }
    }

    void CloudRegister::setSourceCloud (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
        source_cloud_with_normal_ = cloud;
    }

    void CloudRegister::getSourceCloud (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud) const {
        cloud = source_cloud_with_normal_;
    }

    void CloudRegister::setTargetCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        target_cloud_ = cloud;
    }

    void CloudRegister::getTargetCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) const {
        cloud = target_cloud_;
    }

    void CloudRegister::setSourceCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        source_cloud_ = cloud;
    }

    void CloudRegister::getSourceCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) const {
        cloud = source_cloud_;
    }

    void CloudRegister::computeGlobalTransformation (Eigen::Matrix4f pair_transformation) {
        global_transformation_ *= pair_transformation;
    }

    Eigen::Affine3d CloudRegister::getGlobalTransformation () const {
        Eigen::Affine3d global_transformation_affine (global_transformation_.cast<double> ());
        return global_transformation_affine;
    }
}

