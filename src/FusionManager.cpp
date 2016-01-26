#include "FusionManager.h"

#include "CloudNormalProvider.h"

namespace MobileFusion {
    FusionManager::FusionManager()
    : renderer_("compare")
    , registerer_()
    , tsdf_()
    , cloud_dirty_(false)
    , cloud_() {
    }

    FusionManager::~FusionManager() {
    }

    void FusionManager::update() {
        if(!cloud_dirty_) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_;

        pcl::PointCloud<pcl::Normal>::Ptr normal = CloudNormalProvider::computeNormal(cloud, 5);

        registerer_.updateCloud(cloud);

        //for first insertion
        if(!registerer_.registerPossible()) {
            tsdf_.integrateCloud(*cloud, *normal, Eigen::Affine3d::Identity());
        }

        //that after
        if(registerer_.registerPossible()) {
            renderer_.onCloudFrame(registerer_.getTargetDownsampled(), registerer_.getSourceRegistered());
            tsdf_.integrateCloud(*cloud, *normal, registerer_.getAffine3d(registerer_.getIcpTransformation()));
        }
    }

    void FusionManager::onFrame(const cv::Mat& rgb, const cv::Mat& depth) {
            }

    void FusionManager::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        cloud_ = cloud;
        cloud_dirty_ = true;
    }
}

