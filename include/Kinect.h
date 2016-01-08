#ifndef KINECT_H_
#define KINECT_H_

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <pcl/point_types.h>

#include "CameraInterface.h"


namespace mobilefusion {
    class Kinect : public CameraInterface {
        public:
            Kinect(
                    bool bilateralFiltering,
                    bool edgeAwareFiltering,
                    float minDepth,
                    float maxDepth);
            ~Kinect();
            void init();
            void captureImage(cv::Mat &rgb, cv::Mat &depth);

        private:
            libfreenect2::Freenect2 *freenect2_;
            libfreenect2::Freenect2Device *dev_;
            libfreenect2::PacketPipeline *pipeline_;
            libfreenect2::Registration *registration_;
            libfreenect2::SyncMultiFrameListener *listener_;

            bool bilateralFiltering_;
            bool edgeAwareFiltering_;
            float minKinect2Depth_;
            float maxKinect2Depth_;
    };
}

#endif
