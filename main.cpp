#include <boost/thread.hpp>

#include "CloudNormalProvider.h"
#include "CloudProvider.h"
#include "CloudRenderer.h"
#include "CloudRecorder.h"
#include "FusionManager.h"
#include "Kinect.h"
#include "KinectPngReader.h"
#include "KinectRecorder.h"
#include "KinectRenderer.h"

//X11 should be included after Eigen since it defines Success as 0 that collapses Eigen.
#include <X11/Xlib.h>

int main(int argc, char **argv) {
    XInitThreads();

    boost::shared_ptr<MobileFusion::KinectRecorder> recorder(
        new MobileFusion::KinectRecorder("/home/vllab/Desktop/images/rgb/",
                                         "/home/vllab/Desktop/images/depth/"));
    boost::shared_ptr<MobileFusion::KinectRenderer> renderer(new MobileFusion::KinectRenderer());
    boost::shared_ptr<MobileFusion::CloudProvider> cloud_provider(new MobileFusion::CloudProvider());
    boost::shared_ptr<MobileFusion::FusionManager> fusion_manager(
        new MobileFusion::FusionManager("/home/vllab/Desktop/"));
    boost::shared_ptr<MobileFusion::CloudRenderer> cloud_renderer(new MobileFusion::CloudRenderer("cloud"));
    boost::shared_ptr<MobileFusion::CloudRecorder> cloud_recorder(
        new MobileFusion::CloudRecorder("/home/vllab/Desktop/PCDfiles/"));

    //Choose Kinect or PngReader in here.
    boost::shared_ptr<MobileFusion::KinectInterface> kinect_interface(new MobileFusion::Kinect());
    //boost::shared_ptr<MobileFusion::KinectInterface> kinect_interface(new MobileFusion::KinectPngReader());

    kinect_interface->addFrameListener(recorder);
    //kinect_interface->addFrameListener(renderer);
    kinect_interface->addFrameListener(cloud_provider);
    kinect_interface->addFrameListener(fusion_manager);

    cloud_provider->addListener(cloud_renderer);
    cloud_provider->addListener(fusion_manager);
    cloud_provider->addListener(cloud_recorder);

    //start the Kinect thread
    boost::thread kinect_thread(&MobileFusion::KinectInterface::run, kinect_interface);
    //kinect_thread.join();

    while(true) {
        fusion_manager->update();
    }

    return 0;
}
