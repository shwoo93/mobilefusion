#include "KinectRecorder.h"

#include <iostream>

#include "boost/format.hpp"

namespace MobileFusion{
    KinectRecorder::KinectRecorder()
    : frame_count_(0)
    , min_(0)
    , max_(0) {
    }

    KinectRecorder::~KinectRecorder() {
    }

    void KinectRecorder::setMinFrameCount(int min) {
        min_ = min;
    }

    void KinectRecorder::setMaxFrameCount(int max) {
        max_ = max;
    }

    void KinectRecorder::OnFrame(cv::Mat &rgb, cv::Mat &depth) {
        if(frame_count_ >= min_ && frame_count_ <= max_) {
            std::cout << "save image" << std::endl;
            std::string rgb_name = str(boost::format("/home/vllab/Desktop/images/rgb/kinect_rgb%1%.jpg") % frame_count_);
            std::string depth_name = str(boost::format("/home/vllab/Desktop/images/depth/kinect_depth%1%.jpg") % frame_count_);
            imwrite(rgb_name, rgb);
            imwrite(depth_name, depth);
        }

        ++frame_count_;
    }
}
