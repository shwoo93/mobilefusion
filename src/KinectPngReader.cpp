#include "KinectPngReader.h"

#include "boost/format.hpp"

#include "KinectFrameListener.h"

namespace MobileFusion{
    KinectPngReader::KinectPngReader()
    : frame_listeners_ ()
    , stop_ (false)
    , frame_count_ (1) {
    }

    KinectPngReader::~KinectPngReader() {
    }

    void KinectPngReader::run() {
        while (!stop_) {
            std::string rgbimage_ = str(boost::format("/home/vllab/Desktop/images/rgb/kinect_rgb%1%.png") % frame_count_);
            std::string depthimage_ = str(boost::format("/home/vllab/Desktop/images/depth/kinect_depth%1%.png") % frame_count_);

            cv::Mat rgb = cv::imread(rgbimage_, 1);
            cv::Mat depth = cv::imread(depthimage_, 0);

            if (rgb.empty() || depth.empty()) {
                std::cout<<"Could not open or find the image"<<std::endl;
                break;
            }

            for(std::vector<boost::shared_ptr<KinectFrameListener> >::iterator iter = frame_listeners_.begin();
                iter!= frame_listeners_.end(); ++iter) {
                (*iter)->onFrame (rgb, depth);
            };

            frame_count_++;
        }
    }

    void KinectPngReader::stop() {
        stop_ = true;
    }

    void KinectPngReader::addFrameListener (boost::shared_ptr<KinectFrameListener> frame_listener) {
        frame_listeners_.push_back(frame_listener);
    }
}
