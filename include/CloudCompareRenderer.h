#ifndef __CLOUD_COMPARE_RENDERER_H__
#define __CLOUD_COMPARE_RENDERER_H__

#include <string>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

namespace MobileFusion {
    class CloudCompareRenderer {
    public:
        CloudCompareRenderer(std::string name);
        ~CloudCompareRenderer();
        void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2);
    private:
        pcl::visualization::PCLVisualizer viewer_;
    };
}


#endif
