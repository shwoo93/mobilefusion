#ifndef __CLOUD_RECORDER_H__
#define __CLOUD_RECORDER_H__

#include "CloudListener.h"

namespace MobileFusion {
    class CloudRecorder : public CloudListener {
        public:
            CloudRecorder(std::string folder_path);
            ~CloudRecorder();
            void setMinFrameCount(int min);
            void setMaxFrameCount(int max);
            void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        private:
            std::string folder_path_;
            int frame_count_;
            int min_;
            int max_;
    };
}

#endif
