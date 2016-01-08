#include "Kinect.h"
#include "util3d.h"
#include "util3d_registration.h"

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/octree.h>
#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <eigen_extensions/eigen_extensions.h>

#include <pcl/PolygonMesh.h>

#include <vector>

int main(int argc, char **argv)
{
    mobilefusion::Kinect kinect(true, true, 0.5f, 4.5f);

    kinect.init();

    cv::Mat rgb, depth;

    //while(true)
    //{
    //    std::cout << "Capture image" << std::endl;
    //    kinect.captureImage(rgb, depth);
    //    cv::namedWindow("depth");
    //    cv::namedWindow("rgb");

    //    if(!depth.empty())
    //    {
    //        cv::imshow("depth",depth);
    //        cv::imshow("rgb",rgb);
    //    }
    //    cv::waitKey(1);
    //}

    // return 0;

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> point_vec;

    //transformation matrix
    Eigen::Matrix4f T;
    T<< 1, 0 , 0, 0,
        0, 1 , 0, 0,
        0, 0 , 1, 0;

    //tsdf initialize
    cpu_tsdf::TSDFVolumeOctree::Ptr tsdf (new cpu_tsdf::TSDFVolumeOctree);
    tsdf->setGridSize(1.,1.,1.);
    tsdf->setResolution(128,128,128);
    tsdf->setIntegrateColor(true);
    tsdf->reset();


    while(1)
    {
        static int i = 0;
        kinect.captureImage(rgb,depth);
        point_vec.push_back(mobilefusion::util3d::cloudFromRgbd(rgb,depth,256.0f,212.0f,540.686f,540.686f,10)); //cx 256.0f cy 212.0f fx 540.686f fy 540.686f
        if(point_vec.size()>=2)
        {
            pcl::PointCloud<pcl::PointXYZRGB> empty_cloud;
            Eigen::Matrix4d T_4d = T.cast<double>();
            Eigen::Affine3d T_tmp = Eigen::Affine3d(T_4d);
            tsdf->integrateCloud(*point_vec[i],empty_cloud,T_tmp);
            T*=mobilefusion::util3d::transformationIcp(point_vec[i],point_vec[i+1]);
            i++;
        }
        cpu_tsdf::MarchingCubesTSDFOctree mc;
        mc.setInputTSDF(tsdf);
        mc.setMinWeight(2);
        mc.setColorByRGB(true);
        pcl::PolygonMesh mesh;
        mc.reconstruct (mesh);
        pcl::io.savePLYFileBinary (mesh_file, mesh);
    }

    return 0;
}

