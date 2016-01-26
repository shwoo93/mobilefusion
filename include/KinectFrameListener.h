#ifndef __KINECT_FRAME_LISTENER__
#define __KINECT_FRAME_LISTENER__

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace MobileFusion {
    class KinectFrameListener {
        public:
            KinectFrameListener() {};
            virtual ~KinectFrameListener() {};
            virtual void onFrame(cv::Mat &rgb, cv::Mat &depth) = 0;
    };
}



#endif
