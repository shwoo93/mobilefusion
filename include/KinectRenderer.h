#ifndef __KINECT_RENDERER_H__
#define __KINECT_RENDERER_H__

#include "KinectFrameListener.h"

namespace MobileFusion {
    class KinectRenderer : public KinectFrameListener {
        public:
            KinectRenderer();
            ~KinectRenderer();
            void onFrame(const cv::Mat &rgb, const cv::Mat &depth);
    };
}

#endif
