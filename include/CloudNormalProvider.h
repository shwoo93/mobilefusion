#ifndef __CLOUD_NORMAL_PROVIDER_H__
#define __CLOUD_NORMAL_PROVIDER_H__

#include <boost/shared_ptr.hpp>

#include "CloudListener.h"

namespace MobileFusion {
    class CloudNormalListener;

    class CloudNormalProvider : public CloudListener {
        public:
            CloudNormalProvider();
            ~CloudNormalProvider();
            void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
            void addListener(boost::shared_ptr<CloudNormalListener> listener);
        private:
            pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        private:
            int normal_k_search_;
            pcl::PointCloud<pcl::Normal>::Ptr normals_;
            std::vector<boost::shared_ptr<CloudNormalListener> > listeners_;
    };
}

#endif
