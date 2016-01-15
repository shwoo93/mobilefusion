#ifndef _KINECT_RENDERER_H_
#define _KINECT_RENDERER_H_

#include "KinectFrameListener.h"

namespace mobilefusion {
    class KinectRenderer : public KinectFrameListener {
    public:
        KinectRenderer();
        ~KinectRenderer();
        void OnFrame(cv::Mat &rgb, cv::Mat &depth);
    };
}

#endif
