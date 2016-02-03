#include "FusionManager.h"

#include "CloudNormalProvider.h"

#include <pcl/visualization/pcl_visualizer.h>
#include "boost/format.hpp"
namespace MobileFusion {
    FusionManager::FusionManager()
    : renderer_("compare")
    , registerer_()
    , tsdf_()
    , cloud_dirty_(false)
    , cloud_()
    , update_count_ (0)
    , octree_ () {
        octree_.reset();
    }

    FusionManager::~FusionManager() {
    }

    void FusionManager::update() {

        //if(!cloud_dirty_)
        //    return;

        //++update_count_;

        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_;
        //cloud_dirty_ = false;

        //pcl::PointCloud<pcl::Normal>::Ptr normal = CloudNormalProvider::computeNormal(cloud,5);
        //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudConcatenated (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        //pcl::concatenateFields (*cloud, *normal, *cloudConcatenated);

        //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr raytraced (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        ////first time
        //if(update_count_ == 1) {
        //    //tsdf_.integrateCloud (*cloud, *normal); //
        //    //raytraced = tsdf_.renderColoredView (); //
        //    octree_.integrateCloud (*cloud, *normal);
        //    raytraced = octree_.renderColoredView();
        //    registerer_.setSourceCloud (raytraced);
        //    std::cout<<raytraced->size()<<std::endl;
        //}

        //else {
        //    std::cout<<"a"<<std::endl;
        //    registerer_.setTargetCloud (cloudConcatenated);
        //    std::cout<<"cloudconcatenate:"<<cloudConcatenated->size()<<std::endl;
        //    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_downsampled (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        //    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_registered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        //    registerer_.getIcpResultCloud (5.f, cloud_target_downsampled, cloud_source_registered);
        //    renderer_.onCloudFrame(cloud_target_downsampled, cloud_source_registered);
        //    //tsdf_.integrateCloud (*cloud, *normal, registerer_.getCameraPose ().inverse ()); //
        //    octree_.integrateCloud (*cloud, *normal, registerer_.getCameraPose ().inverse());
        //    std::cout<<registerer_.getCameraPose().matrix()<<std::endl;
        //    //raytraced = tsdf_.renderColoredView (registerer_.getCameraPose().inverse ()); //
        //    raytraced = octree_.renderColoredView (registerer_.getCameraPose().inverse ());
        //    std::cout<<"raytraced"<<raytraced->size()<<std::endl;
        //    registerer_.setSourceCloud (raytraced);

        static int index = 1;
        if(!cloud_dirty_)
            return;

        ++update_count_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_));
        cloud_dirty_ = false;

        pcl::PointCloud<pcl::Normal>::Ptr normal = CloudNormalProvider::computeNormal(cloud, 4);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_concatenated (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::concatenateFields (*cloud, *normal, *cloud_concatenated);

        if(update_count_ == 1) {
            std::string count = str(boost::format("%1%") % index);
            tsdf_.integrateCloud (*cloud,*normal);
            //octree_.integrateCloud (*cloud, *normal);
            registerer_.setTargetCloud (cloud_concatenated);
            index++;
        }

        else {
            std::string count = str(boost::format("%1%") % index);
            std::cout<< count <<std::endl;
            registerer_.setSourceCloud (cloud_concatenated);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_downsampled (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_registered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            registerer_.getIcpResultCloud (5.f, cloud_target_downsampled, cloud_source_registered);
            //octree_.integrateCloud (*cloud, *normal, registerer_.getCameraPose());
            tsdf_.integrateCloud (*cloud, *normal, registerer_.getCameraPose());
            registerer_.setTargetCloud (cloud_concatenated);
            index++;
        }

        tsdf_.constructMesh ();
    }

    void FusionManager::onFrame(const cv::Mat& rgb, const cv::Mat& depth) {
    }

    void FusionManager::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        cloud_ = cloud;
        cloud_dirty_ = true;
    }
}

