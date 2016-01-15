#include "Kinect.h"
#include "KinectRecorder.h"
#include "KinectRenderer.h"
#include "KinectFrameToCloud.h"
#include "KinectRegistration.h"

int main(int argc, char **argv) {
    boost::shared_ptr<MobileFusion::KinectRecorder> recorder(new MobileFusion::KinectRecorder());
    boost::shared_ptr<MobileFusion::KinectRenderer> renderer(new MobileFusion::KinectRenderer());
    boost::shared_ptr<MobileFusion::KinectFrameToCloud> converter(new MobileFusion::KinectFrameToCloud());

    recorder->setMinFrameCount(10);
    recorder->setMaxFrameCount(50);

    MobileFusion::Kinect kinect;
    kinect.addFrameListener(recorder);
    kinect.addFrameListener(renderer);
    kinect.addFrameListener(converter);

    while(true) {
    	kinect.updateFrame();
    }

    return 0;
}
