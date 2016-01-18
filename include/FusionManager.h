#ifndef __FUSION_MANAGER_H__
#define __FUSION_MANAGER_H__

#include <pcl/registration/gicp.h>
#include <pcl/registration/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include "CloudListener.h"

namespace MobileFusion {
    class FusionManager : public CloudListener {
        public:
            FusionManager();
            ~FusionManager();
            void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
            void setVoxelSize(float voxelsize);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNaNFromPointCloud(
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
            Eigen::Matrix4f getIcpTransformation(
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_source,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_target);
        private:
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_;
            float voxelsize_;
    };

}
#endif
