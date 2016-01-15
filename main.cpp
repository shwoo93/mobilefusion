<<<<<<< HEAD

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
#include "util3d.h"
#include "util3d_registration.h"

int main(int argc, char **argv)
{
    mobilefusion::Kinect kinect(true, true, 0.5f, 4.5f);
=======
#include "Kinect.h"
#include <list>
#include <vector>

int main(int argc, char **argv)
{
    mobilefusion::Kinect kinect;
>>>>>>> d90abbe570863f5858036b9a744f61a42d295191

    kinect.init();

    cv::Mat rgb, depth;

<<<<<<< HEAD
    //transformation matrix
    Eigen::Matrix4f T;
    T<< 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    cpu_tsdf::TSDFVolumeOctree::Ptr tsdf (new cpu_tsdf::TSDFVolumeOctree);
    tsdf->setGridSize(1.,1.,1.);
    tsdf->setResolution(128,128,128);
    tsdf->setIntegrateColor(true);
    tsdf->reset();

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

=======
    while(true)
    {
        std::cout << "Capture image" << std::endl;
        kinect.captureImage(rgb, depth);
        cv::namedWindow("depth");
        cv::namedWindow("rgb");

        //if(depth.empty())
        //    std::cout<<"depth empty"<<endl;
        //if(rgb.empty())
        //    std::cout<<"rgb empty"<<endl;
>>>>>>> d90abbe570863f5858036b9a744f61a42d295191
        if(!depth.empty())
        {
            cv::imshow("depth",depth);
            cv::imshow("rgb",rgb);
        }
        cv::waitKey(1);
<<<<<<< HEAD

        point_vec.push_back(mobilefusion::util3d::cloudFromRgbd(rgb,depth,256.0f,212.0f,540.686f,540.686f,5));

        if(point_vec.size()>=2)
        {
            Eigen::Matrix4d T_4d = T.cast<double>();
            Eigen::Affine3d T_tmp = Eigen::Affine3d(T_4d);
            transform_list.push_back(T_tmp);
            T*=mobilefusion::util3d::transformationIcp_Xyzrgb(point_vec[i],point_vec[i+1]);
            tsdf->integrateCloud(*point_vec[i], empty_cloud, transform_list[i]);
            i++;
        }

        time++;

    }

    mc.setInputTSDF(tsdf);
    pcl::PolygonMesh mesh;
    mc.reconstruct (mesh);
    pcl::io::savePLYFileBinary ("mesh_file", mesh);
    std::cout<<"end"<<std::endl;

    return 0;

}


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


=======
    }

    return 0;
    //mobilefusion::Kinect* kinect_ptr = new mobilefusion::Kinect();
    //std::vector<pcl::PointCloud<pcl::PointXYZRGB> point_vec;
    //std::list<Eigen::Matrix4f> transform_list;
    //cv::Mat rgb,depth;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    //Eigen::Matrix4f T;
    //T<< 1, 0 , 0, 0,
    //    0, 1 , 0, 0,
    //    0, 0 , 1, 0;

    //transform_list.push_back(T);
    //TSDFVolumeOctree::Ptr tsdf(new TSDFVolumeOctree);
    //tsdf->setGridSize(1.,1.,1.);
    //tsdf->setResolution(128,128,128);
    //tsdf->setIntegrateColor(true);
    //tsdf->reset();

    //while(1) //?? while kinect is connected
    //{
    //    kinect_ptr->capture(rgb,depth);
    //    //cx = 256.0f cy= 212.0f fx= 540.686f fy= 540.686f
    //    point_vec.push_back(cloudFromRgbd(depth,rgb,cx,cy,fx,fy));//cx,cy offset... fx,fy focallength

    //    if(point_list.size()>=2)
    //    {

    //        static int i=0;
    //        Eigen::Matrix4f T_i;
    //        T_i=GetTransMat(&point_vec[i],&point_vec[i+1]);
    //        transform_list.push_back(T_i);
    //        for(;i<point_vec.size();i++)
    //        {
    //            Eigen::Matrix4f T_tmp;
    //            T_tmp = transform_list.pop_front();
    //            tsdf->integrateCloud(point_vec[i],,T_tmp);
    //            transform_list.push_back(T_tmp*transform_list.pop_front());
    //        }
    //    }

    //    MarchingCubesTSDFOctree mc;
    //    mc.setInputTSDF(tsdf);
    //    mc.setMinWeight(2);
    //    mc.setColorByRGB(true);
    //    pcl::PolygonMesh mesh;
    //    mc.reconstruct (mesh);
    //}

}
>>>>>>> d90abbe570863f5858036b9a744f61a42d295191
