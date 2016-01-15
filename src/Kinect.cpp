#include "Kinect.h"

namespace mobilefusion{
    Kinect::Kinect(
            bool bilateralFiltering,
            bool edgeAwareFiltering,
            float minDepth,
            float maxDepth) :
                bilateralFiltering_(bilateralFiltering),
                edgeAwareFiltering_(edgeAwareFiltering),
                minKinect2Depth_(minDepth),
                maxKinect2Depth_(maxDepth),
                freenect2_(),
                dev_(),
                pipeline_(),
                registration_(),
                listener_()
    {
        if (minDepth < 0.5f || maxDepth > 10.0f)
            std::cout << "minDepth must be > 0.5f and maxDepth must be < 10.0f" << std::endl;
    }

    Kinect::~Kinect()
    {
        dev_->stop();
        dev_->close();

        //delete registration_;
    }

    void Kinect::init()
    {
        freenect2_ = boost::shared_ptr<libfreenect2::Freenect2>(new libfreenect2::Freenect2());
        if (freenect2_->enumerateDevices() == 0)
        {
            std::cout << "no device connected!" << std::endl;
            exit(1);
        }

        std::string serial = freenect2_->getDefaultDeviceSerialNumber();
        pipeline_ = boost::shared_ptr<libfreenect2::OpenCLPacketPipeline>(new libfreenect2::OpenCLPacketPipeline());
        dev_ = boost::shared_ptr<libfreenect2::Freenect2Device>(freenect2_->openDevice(serial, pipeline_.get()));


        if (dev_ == 0)
        {
            std::cout << "failure opening device!" << std::endl;
        }

        listener_ = boost::shared_ptr<libfreenect2::SyncMultiFrameListener>(new 
            libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth));

        dev_->setColorFrameListener(listener_.get());
        dev_->setIrAndDepthFrameListener(listener_.get());

        dev_->start();

        std::cout << "device serial: " << dev_->getSerialNumber() << std::endl;
        std::cout << "device firmware: " << dev_->getFirmwareVersion() << std::endl;

        registration_ = boost::shared_ptr<libfreenect2::Registration>(new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams()));

        //libfreenect2::Freenect2Device::Config config;
        //config.EnableBilateralFilter = bilateralFiltering_;
        //config.EnableEdgeAwareFilter = edgeAwareFiltering_;
        //config.MinDepth = minKinect2Depth_;
        //config.MaxDepth = maxKinect2Depth_;
        //dev_->setConfiguration(config);
    }

    void Kinect::captureImage(cv::Mat &rgb, cv::Mat &depth)
    {
        libfreenect2::FrameMap frames;
        listener_->waitForNewFrame(frames);
        libfreenect2::Frame *rgb_frame = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth_frame = frames[libfreenect2::Frame::Depth];
        libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

        registration_->apply(rgb_frame, depth_frame, &undistorted, &registered);


        cv::Mat rgb_temp((int)registered.height, (int)registered.width, CV_8UC4, registered.data);
        cv::Mat depth_temp((int)depth_frame->height, (int)depth_frame->width , CV_32FC1,depth_frame->data);

        rgb = rgb_temp;
        depth = depth_temp;

        listener_->release(frames);
    }

}
