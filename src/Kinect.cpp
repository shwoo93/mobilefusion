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
                freenect2_(0),
                dev_(0),
                pipeline_(0),
                registration_(0),
                listener_(0)
    {
        if (minDepth < 0.5f || maxDepth > 10.0f)
            std::cerr << "minDepth must be > 0.5f and maxDepth must be < 10.0f" << std::endl;
    }

    Kinect::~Kinect()
    {
        dev_->stop();
        dev_->close();

        delete registration_;
    }

    void Kinect::init()
    {
        freenect2_ = new libfreenect2::Freenect2();
        if (freenect2_->enumerateDevices() == 0)
        {
            std::cout << "no device connected!" << std::endl;
            exit(1);
        }

        std::string serial = freenect2_->getDefaultDeviceSerialNumber();
        pipeline_ = new libfreenect2::OpenCLPacketPipeline();
        dev_ = freenect2_->openDevice(serial, pipeline_);


        if (dev_ == 0)
        {
            std::cout << "failure opening device!" << std::endl;
        }

        listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

        dev_->setColorFrameListener(listener_);
        dev_->setIrAndDepthFrameListener(listener_);

        dev_->start();

        std::cout << "device serial: " << dev_->getSerialNumber() << std::endl;
        std::cout << "device firmware: " << dev_->getFirmwareVersion() << std::endl;

        registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

        libfreenect2::Freenect2Device::Config config;
        config.EnableBilateralFilter = bilateralFiltering_;
        config.EnableEdgeAwareFilter = edgeAwareFiltering_;
        config.MinDepth = minKinect2Depth_;
        config.MaxDepth = maxKinect2Depth_;
        dev_->setConfiguration(config);
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
