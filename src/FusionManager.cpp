#include "FusionManager.h"

namespace MobileFusion {
    FusionManager::FusionManager()
    : normalprovider_()
    , renderer_("compare")
    , registerer_()
    , wrapper_() {
    }

    FusionManager::~FusionManager() {
    }

    void FusionManager::onFrame(cv::Mat &rgb, cv::Mat &depth) {
        std::cout<<"FusionManager_onFrame"<<std::endl;
    }

    void FusionManager::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        registerer_.updateCloud(cloud);

        //for first insertion
        if(!registerer_.registerPossible()) {
            wrapper_.integrateCloud(*cloud, *(normalprovider_.getCloudNormal()), Eigen::Affine3d::Identity());
        }

        //that after
        if(registerer_.registerPossible()) {
            renderer_.onCloudFrame(registerer_.getTargetDownsampled(), registerer_.getSourceRegistered());
            wrapper_.integrateCloud(*cloud, *(normalprovider_.getCloudNormal()), registerer_.getAffine3d(registerer_.getIcpTransformation()));
        }

        //wrapper_.constructMesh();
    }
}

