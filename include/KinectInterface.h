#ifndef __KINECT_INTERFACE_H__
#define __KINECT_INTERFACE_H__

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

#include "KinectFrameListener.h"

namespace MobileFusion {

    class KinectInterface {
        public:
            KinectInterface() {};
            virtual ~KinectInterface() {};
            virtual void run() = 0;
            virtual void stop() = 0;
            virtual void addFrameListener(boost::shared_ptr<KinectFrameListener> frame_listener) = 0;
    };
}
#endif
