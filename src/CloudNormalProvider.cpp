#include "CloudNormalProvider.h"

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

namespace MobileFusion {
    pcl::PointCloud<pcl::Normal>::Ptr CloudNormalProvider::computeNormal(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int k_search) {

        //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
        normal_estimation.setSearchMethod(tree);
        normal_estimation.setKSearch(k_search);

        normal_estimation.setInputCloud(cloud);
        normal_estimation.compute(*normals);

        return normals;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr CloudNormalProvider::computePointWithNormal(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int k_search) {

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> normal_estimation;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
        //normal_estimation.setSearchMethod(tree);
        normal_estimation.setKSearch(k_search);

        normal_estimation.setInputCloud(cloud);
        normal_estimation.compute(*points_with_normals);
        pcl::copyPointCloud(*cloud, *points_with_normals);

        return points_with_normals;
    }
}


