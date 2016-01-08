#include "Kinect.h"
#include "util3d.h"
#include "util3d_registration.h"

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/octree.h>
#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <eigen_extensions/eigen_extensions.h>

#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>


#include <vector>
#include <list>

int main(int argc, char **argv)
{
    mobilefusion::Kinect kinect(true, true, 0.5f, 4.5f);

    kinect.init();

    cv::Mat rgb, depth;

    //transformation matrix
    Eigen::Matrix4f T;
    T<< 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    //tsdf initialize
    cpu_tsdf::TSDFVolumeOctree::Ptr tsdf (new cpu_tsdf::TSDFVolumeOctree);
    tsdf->setGridSize(1.,1.,1.);
    tsdf->setResolution(128,128,128);
    tsdf->setIntegrateColor(true);
    tsdf->reset();
    //marchingcube
    cpu_tsdf::MarchingCubesTSDFOctree mc;
    mc.setMinWeight(2);
    mc.setColorByRGB(true);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> point_vec;
    std::vector<Eigen::Affine3d> transform_list;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_before;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_current;

    //kinect.captureImage(rgb,depth);
    //pointcloud_before = mobilefusion::util3d::cloudFromRgbd(rgb,depth,256.0f,212.0f,540.686f,540.686f,1); //cx 256.0f cy 212.0f fx 540.686f fy 540.686f

    //make an empty cloud
    pcl::PointCloud<pcl::PointXYZRGB> empty_cloud;

    cv::namedWindow("rgb");
    cv::namedWindow("depth");

    double time = 0;
    while(time<=100)
    {
        static int i =0;
        std::cout << "CaputreImage()" << std::endl;
        //initialize pointcloud_current
        kinect.captureImage(rgb,depth);

        cv::namedWindow("depth");
        cv::namedWindow("rgb");

        if(!depth.empty())
        {
            cv::imshow("depth",depth);
            cv::imshow("rgb",rgb);
        }
        cv::waitKey(1);

        point_vec.push_back(mobilefusion::util3d::cloudFromRgbd(rgb,depth,256.0f,212.0f,540.686f,540.686f,5));

        if(point_vec.size()>=2)
        {
            Eigen::Matrix4d T_4d = T.cast<double>();
            Eigen::Affine3d T_tmp = Eigen::Affine3d(T_4d);
            transform_list.push_back(T_tmp);
            T*=mobilefusion::util3d::transformationIcp_Xyzrgb(point_vec[i],point_vec[i+1]);
            i++;
        }
        //static int j=0;
        //for(;j<transform_list.size();j++)
        //{
        //    tsdf->integrateCloud(*point_vec[j],empty_cloud,transform_list[j]);
        //}

        time++;

    }

    for(int i=0;i<transform_list.size();i++)
    {
            pcl::PointCloud<pcl::PointXYZRGB> point_cloud = *point_vec[i];
            tsdf->integrateCloud(point_cloud,empty_cloud,transform_list[i]);
    }

    //tsdf->integrateCloud(*point_vec[i],empty_cloud,transform_list[i]);
    //std::cout<<i<<std::endl;

    mc.setInputTSDF(tsdf);
    pcl::PolygonMesh mesh;
    mc.reconstruct (mesh);
    pcl::io::savePLYFileBinary ("mesh_file", mesh);
    std::cout<<"end"<<std::endl;

    return 0;
    //pointcloud_current = mobilefusion::util3d::cloudFromRgbd(rgb,depth,256.0f,212.0f,540.686f,540.686f,1);
    //tsdf->integrateCloud(*pointcloud_before,empty_cloud,T_tmp);
    //T*=mobilefusion::util3d::transformationIcp(pointcloud_before,pointcloud_current);
    //pointcloud_before = pointcloud_current;

}
//cpu_tsdf::MarchingCubesTSDFOctree mc;
//mc.setInputTSDF(tsdf);
//mc.setMinWeight(2);
//mc.setColorByRGB(true);
//pcl::PolygonMesh mesh;
//mc.reconstruct (mesh);
//return 0;
//}


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


