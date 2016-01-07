#include "Kinect.h"
#include <list>
#include <vector>

int main(int argc, char **argv)
{
    mobilefusion::Kinect kinect(true, true, 0.5f, 4.5f);

    kinect.init();

    cv::Mat rgb, depth;

    while(true)
    {
        std::cout << "Capture image" << std::endl;
        kinect.captureImage(rgb, depth);
        cv::namedWindow("depth");
        cv::namedWindow("rgb");

        if(!depth.empty())
        {
            cv::imshow("depth",depth);
            cv::imshow("rgb",rgb);
        }
        cv::waitKey(1);
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
