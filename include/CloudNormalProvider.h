#ifndef __CLOUD_NORMAL_PROVIDER_H__
#define __CLOUD_NORMAL_PROVIDER_H__

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

#include "CloudListener.h"

namespace MobileFusion {
    class CloudNormalProvider : public CloudListener {
        public:
            CloudNormalProvider();
            ~CloudNormalProvider();
            pcl::PointCloud<pcl::Normal>::Ptr getCloudNormal();
            void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
            pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        private:
            int normalKSearch_;
            pcl::PointCloud<pcl::Normal>::Ptr normals_;
    };
}

#endif
