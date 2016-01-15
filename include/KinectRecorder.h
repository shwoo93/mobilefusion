#ifndef _KINECT_RECORDER_H_
#define _KINECT_RECORDER_H_

#include "KinectFrameListener.h"

namespace mobilefusion {
    class KinectRecorder : public KinectFrameListener {
    public:
        KinectRecorder();
        ~KinectRecorder();
        void OnFrame(cv::Mat &rgb, cv::Mat &depth);
    };
}

#endif
