#include <iostream>
#include <vector>

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

#include "CloudProvider.h"
#include "CloudRenderer.h"
#include "Kinect.h"
#include "KinectRecorder.h"
#include "KinectRenderer.h"
#include "FusionManager.h"



int main(int argc, char **argv) {
    //boost::shared_ptr<MobileFusion::KinectRecorder> recorder(new MobileFusion::KinectRecorder());
    boost::shared_ptr<MobileFusion::KinectRenderer> renderer(new MobileFusion::KinectRenderer());
    boost::shared_ptr<MobileFusion::CloudProvider> cloud_provider(new MobileFusion::CloudProvider());
    boost::shared_ptr<MobileFusion::CloudNormalProvider> normal_provider(new MobileFusion::CloudNormalProvider());
    boost::shared_ptr<MobileFusion::FusionManager> fusion_manager(new MobileFusion::FusionManager());
    boost::shared_ptr<MobileFusion::CloudRenderer> cloud_renderer(new MobileFusion::CloudRenderer("cloud"));

    //recorder->setMinFrameCount(10);
    //recorder->setMaxFrameCount(50);

    std::vector<cv::Mat> frames;
    MobileFusion::Kinect kinect;
    //kinect.addFrameListener(recorder);
    kinect.addFrameListener(renderer);
    kinect.addFrameListener(cloud_provider);
    kinect.addFrameListener(fusion_manager);

    cloud_provider->addListener(normal_provider);
    cloud_provider->addListener(fusion_manager);
    cloud_provider->addListener(cloud_renderer);

    //start grabbing task
    boost::thread thread1(&MobileFusion::Kinect::grabFrame, &kinect);
    size_t bufSize;

    while(true) {

        kinect.lockMemory();

        bufSize = kinect.getBufferSize();
        std::cout<<bufSize<<std::endl;
        if(bufSize > 0) {
            frames = kinect.getFrames();
            kinect.releaseBuffer();
        }

        kinect.unlockMemory();

        if(bufSize > 0) {
            kinect.processFrame(frames);
            bufSize--;
        }

        if(kinect.getBufferSize() == 40) {
            std::cout<<"Time of FrameProcessing is too slow"<<std::endl;
            kinect.setGrabOn(false);
            thread1.join();

            while(!kinect.isBufferEmpty()) {
                frames = kinect.getFrames();
                kinect.releaseBuffer();
                kinect.processFrame(frames);
            }
            break;
        }
    }

    //while(true) {
    //	kinect.updateFrame();
    //}

    return 0;
}
