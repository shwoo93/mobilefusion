#ifndef __CLOUD_REGISTER_H__
#define __CLOUD_REGISTER_H__

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
            bool registerPossible();
            void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr getICPReadyCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
            void ICP(
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target);//Affine3d?
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTargetDownsampled();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSourceRegistered();
            Eigen::Matrix4f getIcpTransformation();
            Eigen::Affine3d getAffine3d(Eigen::Matrix4f T);
        private:
            float voxelsize_;
            bool registerpossible_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_;//targetDownsampled
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_;//sourceDownsampled
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> rendererInput_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_registered_;
            Eigen::Matrix4f transformation_;
    };
}
#endif
