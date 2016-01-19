#ifndef __CLOUD_RENDERER_H__
#define __CLOUD_RENDERER_H__

#include <string>

#include <pcl/visualization/cloud_viewer.h>

#include "CloudListener.h"

namespace MobileFusion {
    class CloudRenderer : public CloudListener {
        public:
            CloudRenderer(std::string name);
            ~CloudRenderer();
            void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        private:
            pcl::visualization::PCLVisualizer viewer_;
    };
}

#endif
