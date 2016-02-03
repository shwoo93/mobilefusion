#ifndef __CLOUD_RECORDER_H__
#define __CLOUD_RECORDER_H__

#include "CloudListener.h"

namespace MobileFusion {
    class CloudRecorder : public CloudListener {
        public:
            CloudRecorder();
            ~CloudRecorder();
            void setMinFrameCount(int min);
            void setMaxFrameCount(int max);
            void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        private:
            int frame_count_;
            int min_;
            int max_;
    };
}

#endif
