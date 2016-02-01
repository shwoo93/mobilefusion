#ifndef __FUSION_MANAGER_H__
#define __FUSION_MANAGER_H__

#include "CloudCompareRenderer.h"
#include "CloudListener.h"
#include "CloudRegister.h"
#include "CpuTsdf.h"
#include "KinectFrameListener.h"

#include <boost/thread.hpp>
namespace MobileFusion {
    class FusionManager : public KinectFrameListener, public CloudListener {
        public:
            FusionManager();
            ~FusionManager();
            void update();
            void onFrame(const cv::Mat &rgb, const cv::Mat &depth);
            void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

        private:
            CloudCompareRenderer renderer_;
            CloudRegister registerer_;
            bool cloud_dirty_;
            int update_count_;
            //MarchingCubesTSDFOctree mc_;
            CpuTsdf tsdf_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
            pcl::visualization::PCLVisualizer viewer_;
    };
}
#endif
