#ifndef __KINECT_RECORDER_H__
#define __KINECT_RECORDER_H__

#include "KinectFrameListener.h"

namespace MobileFusion {
    class KinectRecorder : public KinectFrameListener {
        public:
            KinectRecorder(std::string rgb_folder_path, std::string depth_folder_path);
            ~KinectRecorder();
            void setMinFrameCount(int min);
            void setMaxFrameCount(int max);
            void onFrame(const cv::Mat &rgb, const cv::Mat &depth);
        private:
            std::string rgb_folder_path_;
            std::string depth_folder_path_;
            int frame_count_;
            int min_;
            int max_;
    };
}

#endif
