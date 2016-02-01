#ifndef __CLOUD_REGISTER_H__
#define __CLOUD_REGISTER_H__

#include <assert.h>
#include <vector>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>


namespace MobileFusion {
    class CloudRegister {
        public :
            CloudRegister();
            ~CloudRegister();

            void getIcpResultCloud(
                    int voxelsize,
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud_target_downsampled,
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud_source_registered);

            Eigen::Affine3d getCameraPose();

            void setSourceCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

            void setTargetCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

            void getSourceCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud) const;

            void getTargetCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud) const;

        private:
            std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> rendererInput_;

            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source_cloud_;

            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target_cloud_;

            Eigen::Matrix4f global_transform_;
    };
}
#endif
