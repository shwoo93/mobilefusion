#include "FusionManager.h"

#include "CloudNormalProvider.h"

#include <boost/format.hpp>
#include <pcl/PolygonMesh.h>

namespace MobileFusion {
    FusionManager::FusionManager()
    : registerer_()
    , tsdf_wrapper_()
    , cloud_dirty_(false)
    , cloud_()
    , update_count_(0)
    , octree_() {
        octree_.reset();
    }

    FusionManager::~FusionManager() {
    }

    void FusionManager::update() {

        //if(!cloud_dirty_) {
        //    return;
        //}
        //else {
        //    cloud_dirty_ = false;
        //}

        //if(update_count_ == 0) {
        //    //initialize tsdf_volume
        //    tsdf_wrapper_.setGridSize(3., 3., 3.);
        //    tsdf_wrapper_.setResolution(256, 256, 256);
        //    tsdf_wrapper_.setIntegrateColor(true);
        //    tsdf_wrapper_.reset();
        //    tsdf_wrapper_.setImageSize(512, 424);
        //    tsdf_wrapper_.setCameraIntrinsics(540.686f, 540.686f, 256.0f, 212.0f);
        //    //initialize marching_cube_parameters
        //    tsdf_wrapper_.setMinWeight(2);
        //    tsdf_wrapper_.setColorByRGB(true);
        //}

        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_));
        //pcl::PointCloud<pcl::Normal>::Ptr normal = CloudNormalProvider::computeNormal(cloud);

        //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        //bool hasICPConverged = false;
        //Eigen::Matrix4f pair_transformation;

        //if (update_count_ == 0) {
        //    registerer_.setSourceCloud(cloud);
        //    tsdf_wrapper_.integrateCloud (*cloud, *normal);
        //}

        //else if (update_count_ == 1) {
        //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
        //    registerer_.getSourceCloud(source);
        //    CloudRegister::pairAlign(source, cloud, pair_transformation, hasICPConverged);
        //    if (hasICPConverged) {
        //        registerer_.computeGlobalTransformation (pair_transformation);
        //        tsdf_wrapper_.integrateCloud (*cloud, *normal, registerer_.getGlobalTransformation());
        //        registerer_.setSourceCloud (tsdf_wrapper_.renderColoredView(registerer_.getGlobalTransformation()));
        //    }
        //}

        //else {
        //    registerer_.getSourceCloud(source);
        //    CloudRegister::pairAlign(source, cloud, pair_transformation, hasICPConverged);
        //    if (hasICPConverged) {
        //        registerer_.computeGlobalTransformation (pair_transformation);
        //        tsdf_wrapper_.integrateCloud (*cloud, *normal, registerer_.getGlobalTransformation());
        //        registerer_.setSourceCloud (tsdf_wrapper_.renderColoredView(registerer_.getGlobalTransformation()));
        //    }
        //}

        //if (update_count_ > 0) {
        //    tsdf_wrapper_.setInputTSDF ();
        //    pcl::PolygonMesh mesh;
        //    tsdf_wrapper_.reconstruct (mesh);
        //    std::string mesh_name = str(boost::format("/home/vllab/Desktop/mesh_rtf/test%1%") % update_count_);
        //    pcl::io::savePolygonFilePLY(mesh_name, mesh);
        //}

        //++update_count_;

        if(!cloud_dirty_) {
            return;
        }
        else {
            cloud_dirty_ = false;
        }

        if(update_count_ == 0) {
            //initialize tsdf_volume
            tsdf_wrapper_.setGridSize(3., 3., 3.);
            tsdf_wrapper_.setResolution(512, 512, 512);
            tsdf_wrapper_.setIntegrateColor(true);
            tsdf_wrapper_.reset();
            tsdf_wrapper_.setImageSize(512, 424);
            tsdf_wrapper_.setCameraIntrinsics(540.686f, 540.686f, 256.0f, 212.0f);
            //initialize marching_cube_parameters
            tsdf_wrapper_.setMinWeight(2);
            tsdf_wrapper_.setColorByRGB(true);
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_));
        pcl::PointCloud<pcl::Normal>::Ptr normal = CloudNormalProvider::computeNormal(cloud);

        if(update_count_ == 0) {
            registerer_.setTargetCloud(cloud);
            tsdf_wrapper_.integrateCloud (*cloud, *normal);
        }

        else {
            bool hasICPConverged = false;
            Eigen::Matrix4f pair_transformation;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
            registerer_.getTargetCloud(target);
            CloudRegister::pairAlign(cloud, target, pair_transformation, hasICPConverged);
            if(hasICPConverged) {
                registerer_.computeGlobalTransformation (pair_transformation);
                tsdf_wrapper_.integrateCloud (*cloud, *normal, registerer_.getGlobalTransformation());
                registerer_.setTargetCloud(cloud);
            }
        }

        if (update_count_ > 0) {
            tsdf_wrapper_.setInputTSDF ();
            pcl::PolygonMesh mesh;
            tsdf_wrapper_.reconstruct (mesh);
            std::string mesh_name = str(boost::format("/home/vllab/Desktop/mesh_ftf/test%1%") % update_count_);
            pcl::io::savePolygonFilePLY(mesh_name, mesh);
        }

        ++update_count_;
    }

    void FusionManager::onFrame(const cv::Mat& rgb, const cv::Mat& depth) {
    }

    void FusionManager::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        cloud_ = cloud;
        cloud_dirty_ = true;
    }
}

