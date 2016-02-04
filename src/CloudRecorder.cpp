#include "CloudRecorder.h"

#include <iostream>
#include <limits>
#include <sys/stat.h>
#include <unistd.h>

#include <boost/format.hpp>
#include <pcl/io/pcd_io.h>

namespace MobileFusion{
    CloudRecorder::CloudRecorder()
    : frame_count_(0)
    , min_(0)
    , max_(std::numeric_limits<int>::max()) {
    }

    CloudRecorder::~CloudRecorder() {
    }

    void CloudRecorder::setMinFrameCount(int min) {
        min_ = min;
    }

    void CloudRecorder::setMaxFrameCount(int max) {
        max_ = max;
    }

    void CloudRecorder::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        if(frame_count_ >= min_ && frame_count_ <= max_) {
            struct stat st;
            if(stat("/home/vllab/Desktop/PCDfiles/", &st) != 0) {
                std::cerr << "folder missing: /home/vllab/Desktop/PCDfiles/" << std::endl;
                assert(0);
            }
            
            std::string cloud_name = str(boost::format ("/home/vllab/Desktop/PCDfiles/cloud%1%.pcd") % frame_count_);
            pcl::io::savePCDFile(cloud_name, *cloud);
        }

        ++frame_count_;
    }
}
