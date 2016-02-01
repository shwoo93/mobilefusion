#include "FusionManager.h"

#include "CloudNormalProvider.h"

#include <pcl/visualization/pcl_visualizer.h>
namespace MobileFusion {
    FusionManager::FusionManager()
    : renderer_("compare")
    , registerer_()
    , tsdf_()
    , cloud_dirty_(false)
    , cloud_()
    , viewer_("Test")
    , update_count_ (0) {
    }


    FusionManager::~FusionManager() {
    }

    void FusionManager::update() {

        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_;
        //cloud_dirty_ = false;

        //pcl::PointCloud<pcl::Normal>::Ptr normal = CloudNormalProvider::computeNormal(cloud, 5);

        //registerer_.updateCloud(cloud);

        ////pcl::PointCloud<pcl::PointNormal>::Ptr raytraced (new pcl::PointCloud<pcl::PointNormal>);

        //if(!registerer_.registerPossible()) {
        //    tsdf_.integrateCloud(*cloud, *normal);
        //}

        //if(registerer_.registerPossible()) {
        //    renderer_.onCloudFrame(registerer_.getTargetDownsampled(), registerer_.getSourceRegistered());
        //    tsdf_.integrateCloud(*cloud, *normal, registerer_.getAffine3d(registerer_.getIcpTransformation()));
        //}

        if(!cloud_dirty_)
            return;

        ++update_count_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_;
        cloud_dirty_ = false;

        pcl::PointCloud<pcl::Normal>::Ptr normal = CloudNormalProvider::computeNormal(cloud,5);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudConcatenated (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::concatenateFields (*cloud, *normal, *cloudConcatenated);

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr raytraced (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        //first time
        if(update_count_ == 1) {
            tsdf_.integrateCloud (*cloud, *normal);
            raytraced = tsdf_.renderColoredView ();
            registerer_.setSourceCloud (raytraced);
        }

        else {
            registerer_.setTargetCloud (cloudConcatenated);

            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_downsampled (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_downsampled (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

            registerer_.getIcpResultCloud (5.f, cloud_target_downsampled, cloud_source_downsampled);
            //renderer_.oncloudframe(targetdownsampled, sourceregistered)
            tsdf_.integrateCloud (*cloud, *normal, registerer_.getCameraPose ().inverse ());
            raytraced = tsdf_.renderColoredView (registerer_.getCameraPose().inverse ());
            registerer_.setSourceCloud (raytraced);
        }

    }

    void FusionManager::onFrame(const cv::Mat& rgb, const cv::Mat& depth) {
    }

    void FusionManager::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        cloud_ = cloud;
        cloud_dirty_ = true;
    }
}

