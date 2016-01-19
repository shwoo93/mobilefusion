#ifndef __CLOUD_LISTENER_H__
#define __CLOUD_LISTENER_H__

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/octree.h>
#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <eigen_extensions/eigen_extensions.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#include <vector>

namespace MobileFusion {
    class CloudListener {
        public:
            CloudListener() {};
            virtual ~CloudListener() {};
            virtual void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) = 0;
    };
}
#endif
