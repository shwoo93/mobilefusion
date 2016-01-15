#ifndef __KINECT_RECORDER_H__
#define __KINECT_RECORDER_H__

#include "KinectFrameListener.h"

namespace MobileFusion {
    class KinectRecorder : public KinectFrameListener {
        public:
            KinectRecorder();
            ~KinectRecorder();
            void setMinFrameCount(int min);
            void setMaxFrameCount(int max);
            void OnFrame(cv::Mat &rgb, cv::Mat &depth);

        private:
            int frame_count_;
            int min_;
            int max_;
    };
}

#endif
