#ifndef __CLOUD_PROVIDER_H__
#define __CLOUD_PROVIDER_H__

#include "CloudListener.h"
#include "KinectFrameListener.h"

namespace MobileFusion {

    class CloudProvider : public KinectFrameListener {
        public:
            CloudProvider();
            ~CloudProvider();
            void onFrame(cv::Mat &rgb, cv::Mat &depth);
            void addListener(boost::shared_ptr<CloudListener> cloud_listener);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromRgbd(
                    const cv::Mat &rgb,
                    const cv::Mat &depth,
                    float cx, float cy,
                    float fx, float fy,
                    int decimaiton);
        private:
            float cx_;
            float cy_;
            float fx_;
            float fy_;
            int decimation_;
            int normalKSearch_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
            std::vector<boost::shared_ptr<CloudListener> > cloud_listeners_;
    };
}

#endif




