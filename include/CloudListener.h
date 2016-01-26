#ifndef __CLOUD_LISTENER_H__
#define __CLOUD_LISTENER_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace MobileFusion {
    class CloudListener {
        public:
            CloudListener() {};
            virtual ~CloudListener() {};
            virtual void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) = 0;
    };
}
#endif
