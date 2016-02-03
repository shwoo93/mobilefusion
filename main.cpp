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

    boost::shared_ptr<MobileFusion::KinectRecorder> recorder(new MobileFusion::KinectRecorder());
    boost::shared_ptr<MobileFusion::KinectRenderer> renderer(new MobileFusion::KinectRenderer());
    boost::shared_ptr<MobileFusion::CloudProvider> cloud_provider(new MobileFusion::CloudProvider());
    boost::shared_ptr<MobileFusion::FusionManager> fusion_manager(new MobileFusion::FusionManager());
    boost::shared_ptr<MobileFusion::CloudRenderer> cloud_renderer(new MobileFusion::CloudRenderer("cloud"));
    boost::shared_ptr<MobileFusion::CloudRecorder> cloud_recorder(new MobileFusion::CloudRecorder());

    //recorder->setMinFrameCount(10);
    //recorder->setMaxFrameCount(510);

    MobileFusion::Kinect kinect;
    kinect.addFrameListener(recorder);
    kinect.addFrameListener(renderer);
    kinect.addFrameListener(cloud_provider);
    kinect.addFrameListener(fusion_manager);

    //kinect_pngreader
    //MobileFusion::KinectPngReader kinect_pngreader;
    //kinect_pngreader.addFrameListener(renderer);
    //kinect_pngreader.addFrameListener(cloud_provider);
    //kinect_pngreader.addFrameListener(fusion_manager);

    cloud_provider->addListener(cloud_renderer);
    cloud_provider->addListener(fusion_manager);
    cloud_provider->addListener(cloud_recorder);

    //kinect_pngreader.run();
    //boost::thread kinect_png_thread(&MobileFusion::KinectPngReader::run, &kinect_pngreader);
    //kinect_png_thread.join();


    //start the Kinect thread
    boost::thread kinect_thread(&MobileFusion::Kinect::run, &kinect);
    //kinect_thread.join();

    while(true) {
        fusion_manager->update();
    }

    return 0;
}
