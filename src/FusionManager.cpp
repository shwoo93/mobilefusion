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
    , viewer_("Test") {
    }

    FusionManager::~FusionManager() {
    }

    void FusionManager::update() {
        if(!cloud_dirty_) {
            return;
        }

        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_));
        cloud_dirty_ = false;

        pcl::PointCloud<pcl::Normal>::Ptr normal = CloudNormalProvider::computeNormal(cloud, 5);

        registerer_.updateCloud(cloud);

        //pcl::PointCloud<pcl::PointNormal>::Ptr raytraced (new pcl::PointCloud<pcl::PointNormal>);
        //for first insertion
        if(!registerer_.registerPossible()) {
            tsdf_.integrateCloud(*cloud, *normal, Eigen::Affine3d::Identity());
            // tsdfoctree_.integrateCloud(*cloud, *normal, Eigen::Affine3d::Identity());
        }

        //that after
        if(registerer_.registerPossible()) {
            renderer_.onCloudFrame(registerer_.getTargetDownsampled(), registerer_.getSourceRegistered());
            tsdf_.integrateCloud(*cloud, *normal, registerer_.getAffine3d(registerer_.getIcpTransformation()));
        }

        std::cout<<"out out"<<std::endl;
        //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr raytraced =  tsdf_.renderColoredView (registerer_.getAffine3d(registerer_.getIcpTransformation()),1);
        //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(raytraced);
        //viewer_.addPointCloud<pcl::PointXYZRGBNormal> (raytraced,"test");
        //viewer_.spinOnce();
        //viewer_.spinOnce(1);
        //viewer_.removePointCloud("test");
        //tsdf_.constructMesh();
    }

    void FusionManager::onFrame(const cv::Mat& rgb, const cv::Mat& depth) {
    }

    void FusionManager::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        cloud_ = cloud;
        cloud_dirty_ = true;
    }
}

