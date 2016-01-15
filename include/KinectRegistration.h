#ifndef __KINECT_REGISTRATION_H__
#define __KINECT_REGISTRATION_H__

#include <pcl/registration/gicp.h>
#include <pcl/registration/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>


namespace MobileFusion {
    class KinectRegistration {
        public:
            KinectRegistration();
            ~KinectRegistration();
            void setVoxelSize(float voxelsize);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNaNFromPointCloud(
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
            Eigen::Matrix4f getIcpTransformation(
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_source,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_target);
        private:
            float voxelsize_;
    };

}
#endif
