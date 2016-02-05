#include "FusionManager.h"

#include "CloudNormalProvider.h"

#include <boost/format.hpp>
#include <boost/chrono.hpp>

#include <pcl/PolygonMesh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/octree.h>
#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <eigen_extensions/eigen_extensions.h>

namespace MobileFusion {
    FusionManager::FusionManager()
    : renderer_("compare")
    , registerer_()
    , tsdf_wrapper_()
    , cloud_dirty_(false)
    , cloud_()
    , update_count_ (0) {
    }

    FusionManager::~FusionManager() {
    }

    void FusionManager::update() {

        if(!cloud_dirty_) {
            return;
        }
        else {
            cloud_dirty_ = false;
        }

        if(update_count_ == 0) {
            //initialize tsdf_volume
            tsdf_wrapper_.setGridSize(4., 4., 4.);
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
            CloudRegister::pairAlign(cloud, target, pair_transformation, hasICPConverged); //registerer_get targetcloud
            if(hasICPConverged) {
                registerer_.computeGlobalTransformation (pair_transformation);
                tsdf_wrapper_.integrateCloud (*cloud, *normal, registerer_.getGlobalTransformation());
                registerer_.setTargetCloud(cloud);
            }
        }

        ++update_count_;

        tsdf_wrapper_.setInputTSDF ();
        pcl::PolygonMesh mesh;
        tsdf_wrapper_.reconstruct (mesh);
        std::string mesh_name = str(boost::format("/home/vllab/Desktop/mesh/test%1%") % update_count_);
        pcl::io::savePolygonFilePLY(mesh_name, mesh);
    }

    void FusionManager::onFrame(const cv::Mat& rgb, const cv::Mat& depth) {
    }

    void FusionManager::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        cloud_ = cloud;
        cloud_dirty_ = true;
    }
}

