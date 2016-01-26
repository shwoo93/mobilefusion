#include "FusionManager.h"

namespace MobileFusion {
    FusionManager::FusionManager()
    : renderer_("compare")
    , registerer_()
    , wrapper_()
    , cloud_() {
    }

    FusionManager::~FusionManager() {
    }

    void FusionManager::onFrame(cv::Mat &rgb, cv::Mat &depth) {
        std::cout<<"FusionManager_onFrame"<<std::endl;
    }

    void FusionManager::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        cloud_ = cloud;
    }
    
    void FusionManager::onCloudNormalFrame(pcl::PointCloud<pcl::Normal>::Ptr normal) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_;

        registerer_.updateCloud(cloud);

        //for first insertion
        if(!registerer_.registerPossible()) {
            wrapper_.integrateCloud(*cloud, *normal, Eigen::Affine3d::Identity());
        }

        //that after
        if(registerer_.registerPossible()) {
            renderer_.onCloudFrame(registerer_.getTargetDownsampled(), registerer_.getSourceRegistered());
            wrapper_.integrateCloud(*cloud, *normal, registerer_.getAffine3d(registerer_.getIcpTransformation()));
        }

        //wrapper_.constructMesh();
    }
}

