#ifndef __KINECT_PNG_READER_H__
#define __KINECT_PNG_READER_H__

#include <iostream>
#include <fstream>

#include <string>

#include "KinectFrameListener.h"
#include "KinectInterface.h"

namespace MobileFusion {

    class KinectPngReader : public KinectInterface {
        public:
            KinectPngReader();
            ~KinectPngReader();
            void run ();
            void stop ();
            void addFrameListener (boost::shared_ptr<KinectFrameListener> frame_listener);

        private:
            bool is_file_exist (const std::string& name);
            std::vector<boost::shared_ptr<KinectFrameListener> > frame_listeners_;
            int frame_count_;
            bool stop_;
    };
}
#endif
