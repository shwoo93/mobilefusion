#include <vector>
#include <list>

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/octree.h>
#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <eigen_extensions/eigen_extensions.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>

#include "Kinect.h"
#include "KinectRecorder.h"
#include "KinectRenderer.h"
#include "Util3d.h"
#include "Util3dRegistration.h"

int main(int argc, char **argv)
{
    boost::shared_ptr<mobilefusion::KinectRecorder> recorder(new mobilefusion::KinectRecorder());
    boost::shared_ptr<mobilefusion::KinectRenderer> renderer(new mobilefusion::KinectRenderer());

    mobilefusion::Kinect kinect;
    kinect.addFrameListener(recorder);
    kinect.addFrameListener(renderer);

    while(true)
    {
        kinect.captureImage();
    }

    return 0;

}