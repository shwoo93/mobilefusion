#include "Kinect.h"

#include "KinectFrameListener.h"

namespace MobileFusion{
    Kinect::Kinect()
        : freenect2_()
          , listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth)
          , pipeline_(new libfreenect2::CpuPacketPipeline())
          , dev_(freenect2_.openDevice(freenect2_.getDefaultDeviceSerialNumber(), pipeline_.get()))
          , registration_()
          , frame_listeners_() {
              if (freenect2_.enumerateDevices() == 0) {
                  std::cout << "no device connected!" << std::endl;
                  exit(1);
              }

              if (dev_ == 0) {
                  std::cout << "failure opening device!" << std::endl;
                  exit(1);
              }

              std::cout << "device serial: " << dev_->getSerialNumber() << std::endl;
              std::cout << "device firmware: " << dev_->getFirmwareVersion() << std::endl;

              dev_->setColorFrameListener(&listener_);
              dev_->setIrAndDepthFrameListener(&listener_);

              dev_->start();

              registration_ = boost::shared_ptr<libfreenect2::Registration>(
                      new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams()));
          }

    Kinect::~Kinect() {
        if(dev_ != 0) {
            dev_->stop();
            dev_->close();
        }
    }

    void Kinect::updateFrame() {
        libfreenect2::FrameMap frames;
        listener_.waitForNewFrame(frames);
        libfreenect2::Frame *rgb_frame = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth_frame = frames[libfreenect2::Frame::Depth];
        libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

        registration_->apply(rgb_frame, depth_frame, &undistorted, &registered);

        cv::Mat rgb((int)registered.height, (int)registered.width, CV_8UC4, registered.data);
        cv::Mat depth((int)depth_frame->height, (int)depth_frame->width , CV_32FC1,depth_frame->data);

        listener_.release(frames);

        for(int i = 0; i < frame_listeners_.size(); ++i) {
            frame_listeners_[i]->OnFrame(rgb, depth);
        }
    }

    void Kinect::addFrameListener(boost::shared_ptr<KinectFrameListener> frame_listener) {
        frame_listeners_.push_back(frame_listener);
    }

}
