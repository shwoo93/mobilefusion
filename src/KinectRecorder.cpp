#include "KinectRecorder.h"

#include <iostream>
#include <limits>
#include <sys/stat.h>
#include <unistd.h>

#include "boost/format.hpp"

namespace MobileFusion{
    KinectRecorder::KinectRecorder(std::string rgb_folder_path, std::string depth_folder_path)
    : rgb_folder_path_(rgb_folder_path)
    , depth_folder_path_(depth_folder_path)
    , frame_count_(0)
    , min_(0)
    , max_(std::numeric_limits<int>::max()) {
    }

    KinectRecorder::~KinectRecorder() {
    }

    void KinectRecorder::setMinFrameCount(int min) {
        min_ = min;
    }

    void KinectRecorder::setMaxFrameCount(int max) {
        max_ = max;
    }

    void KinectRecorder::onFrame(const cv::Mat &rgb, const cv::Mat &depth) {
        if(frame_count_ >= min_ && frame_count_ <= max_) {

            struct stat st;
            if(stat(rgb_folder_path_.c_str(), &st) != 0) {
                std::cerr << std::string("folder missing: ") + rgb_folder_path_ << std::endl;
                assert(0);
            }
            if(stat(depth_folder_path_.c_str(), &st) != 0) {
                std::cerr << std::string("folder missing: ") + depth_folder_path_ << std::endl;
                assert(0);
            }

            int index = frame_count_ - min_ + 1;
            std::string rgb_name = rgb_folder_path_ + str(boost::format("kinect_rgb%1%.png") % index);
            std::string depth_name = depth_folder_path_ + str(boost::format("kinect_depth%1%.png") % index);
            imwrite(rgb_name, rgb);
            imwrite(depth_name, depth);
        }

        ++frame_count_;
    }
}
