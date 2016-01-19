#ifndef __KINECT_RENDERER_H__
#define __KINECT_RENDERER_H__

#include "KinectFrameListener.h"

namespace MobileFusion {
    class KinectRenderer : public KinectFrameListener {
        public:
            KinectRenderer();
            ~KinectRenderer();
            void onFrame(cv::Mat &rgb, cv::Mat &depth);
    };
}

#endif
