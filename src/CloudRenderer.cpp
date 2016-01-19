#include "CloudRenderer.h"

namespace MobileFusion {
    CloudRenderer::CloudRenderer(std::string name)
    : viewer_(name) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty(new pcl::PointCloud<pcl::PointXYZRGB>());
        viewer_.addPointCloud(empty, "point cloud");
    };

    CloudRenderer::~CloudRenderer() {
    };

    void CloudRenderer::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer_.updatePointCloud(cloud, rgb, "point cloud");
        viewer_.spinOnce(1);
    }
}
