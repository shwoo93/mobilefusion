#include "CloudProvider.h"
#include "Kinect.h"
#include "KinectRecorder.h"
#include "KinectRenderer.h"
#include "FusionManager.h"

int main(int argc, char **argv) {
    boost::shared_ptr<MobileFusion::KinectRecorder> recorder(new MobileFusion::KinectRecorder());
    boost::shared_ptr<MobileFusion::KinectRenderer> renderer(new MobileFusion::KinectRenderer());
    boost::shared_ptr<MobileFusion::CloudProvider> cloud_provider(new MobileFusion::CloudProvider());
    boost::shared_ptr<MobileFusion::FusionManager> fusion_manager(new MobileFusion::FusionManager());

    recorder->setMinFrameCount(10);
    recorder->setMaxFrameCount(50);

    MobileFusion::Kinect kinect;
    kinect.addFrameListener(recorder);
    kinect.addFrameListener(renderer);
    kinect.addFrameListener(cloud_provider);
    //kincet.addFrameListener(manager);

    cloud_provider->addListener(fusion_manager);

    while(true) {
    	kinect.updateFrame();
    }

    return 0;
}
