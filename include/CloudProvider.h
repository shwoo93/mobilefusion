#ifndef __CLOUD_PROVIDER_H__
#define __CLOUD_PROVIDER_H__

#include <iostream>
#include <pcl/io/pcd_io.h>

#include "KinectFrameListener.h"

namespace MobileFusion {
    class CloudListener;

    class CloudProvider : public KinectFrameListener {
        public:
            CloudProvider();
            ~CloudProvider();
            void onFrame(const cv::Mat& rgb, const cv::Mat& depth);
            void addListener(boost::shared_ptr<CloudListener> cloud_listener);

        private:
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromRgbd(const cv::Mat& rgb, const cv::Mat& depth);

        private:
            float cx_;
            float cy_;
            float fx_;
            float fy_;
            std::vector<boost::shared_ptr<CloudListener> > cloud_listeners_;
    };
}

#endif




