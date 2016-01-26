#ifndef __FUSION_MANAGER_H__
#define __FUSION_MANAGER_H__

#include <pcl/registration/gicp.h>
#include <pcl/registration/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include "CloudCompareRenderer.h"
#include "CloudListener.h"
#include "CloudNormalProvider.h"
#include "CloudRegister.h"
#include "CpuTsdf.h"
#include "KinectFrameListener.h"
#include "CloudNormalListener.h"

namespace MobileFusion {
    class FusionManager : public KinectFrameListener, public CloudListener, public CloudNormalListener {
        public:
            FusionManager();
            ~FusionManager();
            void onFrame(cv::Mat &rgb, cv::Mat &depth);
            void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
            void onCloudNormalFrame(pcl::PointCloud<pcl::Normal>::Ptr normal);
        private:
            CloudCompareRenderer renderer_;
            CloudRegister registerer_;
            CpuTsdf wrapper_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    };
}
#endif
