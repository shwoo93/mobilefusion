#ifndef __KINECT_FRAME_LISTENER__
#define __KINECT_FRAME_LISTENER__

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/octree.h>
#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <eigen_extensions/eigen_extensions.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

namespace MobileFusion {
    class KinectFrameListener {
        public:
            KinectFrameListener() {};
            virtual ~KinectFrameListener() {};
            virtual void OnFrame(cv::Mat &rgb, cv::Mat &depth) = 0;
            //virtual void OnCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) = 0;
    };
}



#endif
