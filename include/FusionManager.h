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
#include "CloudRegister.h"
#include "KinectFrameListener.h"

namespace MobileFusion {
    class FusionManager : public CloudListener, public KinectFrameListener {
        public:
            FusionManager();
            ~FusionManager();
            void onFrame(cv::Mat &rgb, cv::Mat &depth);
            void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        private:
            CloudCompareRenderer renderer_;
            CloudRegister registerer_;
            cpu_tsdf::TSDFVolumeOctree::Ptr tsdf_;
    };
}
#endif
