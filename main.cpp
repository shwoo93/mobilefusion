#include "Kinect.h"
#include "KinectRecorder.h"
#include "KinectRenderer.h"

int main(int argc, char **argv) {
    boost::shared_ptr<MobileFusion::KinectRecorder> recorder(new MobileFusion::KinectRecorder());
    boost::shared_ptr<MobileFusion::KinectRenderer> renderer(new MobileFusion::KinectRenderer());

    recorder->setMinFrameCount(10);
    recorder->setMaxFrameCount(50);

    MobileFusion::Kinect kinect;
    kinect.addFrameListener(recorder);
    kinect.addFrameListener(renderer);

    while(true) {
    	kinect.updateFrame();
    }

    return 0;
}