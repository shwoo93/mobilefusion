#include "CloudCompareRenderer.h"

namespace MobileFusion {
    CloudCompareRenderer::CloudCompareRenderer(std::string name)
    : viewer_(name) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty(new pcl::PointCloud<pcl::PointXYZRGB>());
        viewer_.addPointCloud(empty, "target");
        viewer_.addPointCloud(empty, "registered");
    }

    CloudCompareRenderer::~CloudCompareRenderer() {
    }

    void CloudCompareRenderer::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_targetDownsampled, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_registered) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color_1(cloud_targetDownsampled, 255, 0 , 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color_2(cloud_source_registered, 0, 255, 0);
        viewer_.updatePointCloud(cloud_targetDownsampled, single_color_1, "target");
        viewer_.updatePointCloud(cloud_source_registered, single_color_2, "registered");
        viewer_.spinOnce(1);
    }
}
