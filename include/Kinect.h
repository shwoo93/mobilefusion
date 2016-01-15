#ifndef KINECT_H_
#define KINECT_H_

#include <vector>

#include <boost/shared_ptr.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <pcl/point_types.h>

#include "KinectFrameListener.h"

namespace mobilefusion {
    class Kinect {
        public:
            Kinect();
            ~Kinect();
            void init();
            void captureImage();
            void addFrameListener(boost::shared_ptr<KinectFrameListener> frame_listener);

        private:
            libfreenect2::Freenect2 freenect2_;
            boost::shared_ptr<libfreenect2::PacketPipeline> pipeline_;
            boost::shared_ptr<libfreenect2::Freenect2Device> dev_;
            boost::shared_ptr<libfreenect2::Registration> registration_;
            boost::shared_ptr<libfreenect2::SyncMultiFrameListener> listener_;

            std::vector<boost::shared_ptr<KinectFrameListener> > frame_listeners_;
    };
}

#endif
