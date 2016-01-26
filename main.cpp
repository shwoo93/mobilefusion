#include <boost/thread.hpp>

#include "CloudNormalProvider.h"
#include "CloudProvider.h"
#include "CloudRenderer.h"
#include "FusionManager.h"
#include "Kinect.h"
#include "KinectRecorder.h"
#include "KinectRenderer.h"

int main(int argc, char **argv) {
    //boost::shared_ptr<MobileFusion::KinectRecorder> recorder(new MobileFusion::KinectRecorder());
    boost::shared_ptr<MobileFusion::KinectRenderer> renderer(new MobileFusion::KinectRenderer());
    //boost::shared_ptr<MobileFusion::CloudProvider> cloud_provider(new MobileFusion::CloudProvider());
    //boost::shared_ptr<MobileFusion::CloudNormalProvider> normal_provider(new MobileFusion::CloudNormalProvider());
    //boost::shared_ptr<MobileFusion::FusionManager> fusion_manager(new MobileFusion::FusionManager());
    //boost::shared_ptr<MobileFusion::CloudRenderer> cloud_renderer(new MobileFusion::CloudRenderer("cloud"));

    //recorder->setMinFrameCount(10);
    //recorder->setMaxFrameCount(50);

    MobileFusion::Kinect kinect;
    //kinect.addFrameListener(recorder);
    kinect.addFrameListener(renderer);
    //kinect.addFrameListener(cloud_provider);
    //kinect.addFrameListener(fusion_manager);

    //cloud_provider->addListener(normal_provider);
    //cloud_provider->addListener(fusion_manager);
    //cloud_provider->addListener(cloud_renderer);

    //start the Kinect thread
    boost::thread kinect_thread(&MobileFusion::Kinect::run, &kinect);

    while(true) {
    }

    return 0;
}
