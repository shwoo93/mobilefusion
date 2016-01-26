#ifndef __KINECT_H__
#define __KINECT_H__

#include <queue>
#include <vector>

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

namespace MobileFusion {
    class KinectFrameListener;

    class Kinect {
        public:
            Kinect();
            ~Kinect();
            void init();
            //void updateFrame();
            void addFrameListener(boost::shared_ptr<KinectFrameListener> frame_listener);

            void setGrabOn(bool a);
            size_t getBufferSize();
            std::vector<cv::Mat> getFrames();
            void releaseBuffer();
            void lockMemory();
            void unlockMemory();
            bool isBufferEmpty();
            void grabFrame();
            void processFrame(std::vector<cv::Mat>& imgs);

        private:
            std::queue<std::vector<cv::Mat> > buffer_;
            std::vector<cv::Mat> matbuff_;
            boost::mutex mtxcam_;
            bool grab_;
            //boost::mutex mtx_;
            libfreenect2::Freenect2 freenect2_;
            libfreenect2::SyncMultiFrameListener listener_;
            boost::shared_ptr<libfreenect2::PacketPipeline> pipeline_;
            boost::shared_ptr<libfreenect2::Freenect2Device> dev_;
            boost::shared_ptr<libfreenect2::Registration> registration_;
            std::vector<boost::shared_ptr<KinectFrameListener> > frame_listeners_;
    };
}

#endif
