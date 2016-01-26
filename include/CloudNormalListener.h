#ifndef __CLOUD_NORMAL_LISTENER_H__
#define __CLOUD_NORMAL_LISTENER_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace MobileFusion {
    class CloudNormalListener {
        public:
            CloudNormalListener() {};
            virtual ~CloudNormalListener() {};
            virtual void onCloudNormalFrame(pcl::PointCloud<pcl::Normal>::Ptr normal) = 0;
    };
}
#endif