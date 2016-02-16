#ifndef __FUSION_MANAGER_H__
#define __FUSION_MANAGER_H__

#include <boost/thread.hpp>

#include "CloudCompareRenderer.h"
#include "CloudListener.h"
#include "CloudRegister.h"
#include "KinectFrameListener.h"
#include "TSDFVolumeWrapper.h"

#include "TsdfVolumeOctree.h"

namespace MobileFusion {
    class FusionManager : public KinectFrameListener, public CloudListener {
        public:
            FusionManager();
            ~FusionManager();
            void update();
            void onFrame(const cv::Mat &rgb, const cv::Mat &depth);
            void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

        private:
            //CloudCompareRenderer renderer_;
            CloudRegister registerer_;
            TSDFVolumeWrapper tsdf_wrapper_;
            bool cloud_dirty_;
            int update_count_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
            TSDFVolumeOctree octree_;
    };
}
#endif
