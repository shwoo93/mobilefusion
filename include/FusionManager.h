#ifndef __FUSION_MANAGER_H__
#define __FUSION_MANAGER_H__

#include "CloudCompareRenderer.h"
#include "CloudListener.h"
#include "CloudRegister.h"
#include "CpuTsdf.h"
#include "KinectFrameListener.h"
#include "CloudNormalListener.h"

namespace MobileFusion {
    class FusionManager : public KinectFrameListener, public CloudListener, public CloudNormalListener {
        public:
            FusionManager();
            ~FusionManager();
            void onFrame(const cv::Mat &rgb, const cv::Mat &depth);
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
