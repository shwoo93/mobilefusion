#include "CloudNormalProvider.h"

namespace MobileFusion {

    CloudNormalProvider::CloudNormalProvider()
    : normalKSearch_(4)
    , normals_(new pcl::PointCloud<pcl::Normal>) {
    }

    CloudNormalProvider::~CloudNormalProvider() {
    }

    pcl::PointCloud<pcl::Normal>::Ptr CloudNormalProvider::getCloudNormal() {
        return normals_;
    }

    void CloudNormalProvider::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        normals_ = this->computeNormals(cloud);
    }

    pcl::PointCloud<pcl::Normal>::Ptr CloudNormalProvider::computeNormals(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setKSearch(normalKSearch_);
        n.compute(*normals);

        return normals;
    }

}
