#ifndef __CLOUD_NORMAL_PROVIDER_H__
#define __CLOUD_NORMAL_PROVIDER_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace MobileFusion {
    class CloudNormalProvider {
        private:
            CloudNormalProvider();
        public :
            static pcl::PointCloud<pcl::Normal>::Ptr computeNormal(
                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int k_search = 30);
            static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr computePointWithNormal(
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int k_search = 30);
    };
}

#endif
