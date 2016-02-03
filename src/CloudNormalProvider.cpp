#include "CloudNormalProvider.h"

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

namespace MobileFusion {
    pcl::PointCloud<pcl::Normal>::Ptr CloudNormalProvider::computeNormal(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int k_search) {
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

        std::cout << "CloudNormalProvider::computeNormal() 1" << std::endl;

        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
        std::cout << "CloudNormalProvider::computeNormal() 2" << std::endl;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        std::cout << "CloudNormalProvider::computeNormal() 3" << std::endl;
        normal_estimation.setInputCloud(cloud);
        std::cout << "CloudNormalProvider::computeNormal() 4" << std::endl;
        normal_estimation.setSearchMethod(tree);
        std::cout << "CloudNormalProvider::computeNormal() 5" << std::endl;
        normal_estimation.setKSearch(k_search);
        std::cout << "CloudNormalProvider::computeNormal() 6" << std::endl;
        normal_estimation.compute(*normals);

        std::cout << "CloudNormalProvider::computeNormal() 7" << std::endl;

        return normals;
    }

}
