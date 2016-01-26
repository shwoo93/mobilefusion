#include "CloudNormalProvider.h"

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

#include "CloudNormalListener.h"

namespace MobileFusion {

    CloudNormalProvider::CloudNormalProvider()
    : normal_k_search_(4)
    , listeners_() {
    }

    CloudNormalProvider::~CloudNormalProvider() {
    }

    void CloudNormalProvider::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        pcl::PointCloud<pcl::Normal>::Ptr normal = this->computeNormals(cloud);
        for(std::vector<boost::shared_ptr<CloudNormalListener> >::iterator iter = listeners_.begin() ; iter != listeners_.end() ; iter++) {
            (*iter)->onCloudNormalFrame(normal);
        }
    }

    pcl::PointCloud<pcl::Normal>::Ptr CloudNormalProvider::computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setKSearch(normal_k_search_);
        n.compute(*normals);

        return normals;
    }

}
