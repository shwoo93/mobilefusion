#include "FusionManager.h"

#include <vector>

namespace MobileFusion {
    FusionManager::FusionManager()
    : renderer_("compare")
    , registerer_()
    , tsdf_(new cpu_tsdf::TSDFVolumeOctree) {
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
        pcl::PointCloud<pcl::PointXYZRGB> empty_cloud;
        registerer_.updateCloud(cloud);

        //for first insertion
        if(!registerer_.registerPossible()) {
            tsdf_->integrateCloud(*cloud,empty_cloud);
        }

        //that after
        if(registerer_.registerPossible()) {
            renderer_.onCloudFrame(registerer_.getTargetDownsampled(), registerer_.getSourceRegistered());
            tsdf_->integrateCloud(*cloud,empty_cloud,registerer_.getAffine3d(registerer_.getIcpTransformation()));
        }
    }
}
