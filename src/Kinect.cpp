#include "Kinect.h"

#include <iostream>

#include <boost/chrono.hpp>

#include "KinectFrameListener.h"

namespace MobileFusion{
    Kinect::Kinect()
    : freenect2_()
    , listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth)
    , pipeline_(new libfreenect2::OpenGLPacketPipeline())
    , dev_(freenect2_.openDevice(freenect2_.getDefaultDeviceSerialNumber(), pipeline_.get()))
    , registration_()
    , frame_listeners_()
    , stop_(false) {
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

    void Kinect::run() {
        while(!stop_) {
            libfreenect2::FrameMap frames;
            listener_.waitForNewFrame(frames);
            libfreenect2::Frame *rgb_frame = frames[libfreenect2::Frame::Color];
            libfreenect2::Frame *depth_frame = frames[libfreenect2::Frame::Depth];
            libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

            registration_->apply(rgb_frame, depth_frame, &undistorted, &registered);

            cv::Mat rgb;
            cv::Mat rgba((int)registered.height, (int)registered.width, CV_8UC4, registered.data);
            cv::Mat depth((int)undistorted.height, (int)undistorted.width , CV_32FC1, undistorted.data);
            cv::cvtColor(rgba, rgb, CV_BGRA2BGR);

            listener_.release(frames);


            for(std::vector<boost::shared_ptr<KinectFrameListener> >::iterator iter = frame_listeners_.begin(); iter!=frame_listeners_.end() ; iter++) {
                (*iter)->onFrame(rgb, depth);
            }
        }
    }

    void Kinect::stop() {
        stop_ = true;
    }

    void Kinect::addFrameListener(boost::shared_ptr<KinectFrameListener> frame_listener) {
        frame_listeners_.push_back(frame_listener);
    }
}
