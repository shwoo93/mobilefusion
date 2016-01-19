#ifndef __FUSION_MANAGER_H__
#define __FUSION_MANAGER_H__

#include <pcl/registration/gicp.h>
#include <pcl/registration/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include "CloudCompareRenderer.h"
#include "CloudListener.h"
#include "KinectFrameListener.h"

namespace MobileFusion {
    class FusionManager : public CloudListener, public KinectFrameListener {
        public:
            FusionManager();
            ~FusionManager();
            void onFrame(cv::Mat &rgb, cv::Mat &depth);
            void onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
            Eigen::Matrix4f getIcpTransformation(
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_source,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_target);
        private:
            CloudCompareRenderer renderer_;
            float voxelsize_;
            Eigen::Matrix4f mat_;
            pcl::PointCloud<pcl::PointXYZRGB> empty_cloud_;
            cpu_tsdf::TSDFVolumeOctree::Ptr tsdf_;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_;
    };

}
#endif
