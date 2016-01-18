#ifndef __CLOUD_PROVIDER_H__
#define __CLOUD_PROVIDER_H__

#include "KinectFrameListener.h"
#include "CloudListener.h"

namespace MobileFusion {

    class CloudProvider :public KinectFrameListener {
        public:
            CloudProvider();
            ~CloudProvider();
            void setCameraIntrinsic(float cx, float cy, float fx, float fy);
            void setDecimation(int decimation);
            void OnFrame(cv::Mat &rgb, cv::Mat &depth);
            void addListener(boost::shared_ptr<CloudListener> cloud_listener);
        private:
            float cx_;
            float cy_;
            float fx_;
            float fy_;
            int decimation_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
            std::vector<boost::shared_ptr<CloudListener> > cloud_listeners_;
    };

    namespace FrameToCloud {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromRgbd(
                const cv::Mat &rgb,
                const cv::Mat &depth,
                float cx, float cy,
                float fx, float fy,
                int decimation);
    }
}

#endif

