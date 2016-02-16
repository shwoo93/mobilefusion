#ifndef __CLOUD_REGISTER_H__
#define __CLOUD_REGISTER_H__

#include <assert.h>
#include <vector>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
//TODO: should handle pcl version problem first
//#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>


namespace MobileFusion {
    class CloudRegister {
        public :
            CloudRegister();
            ~CloudRegister();

            static void pairAlign(
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_src,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt,
                    Eigen::Matrix4f& final_transform,
                    bool& hasICPConverged,
                    bool downsample = true);

            static void pairAlign(
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src,
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt,
                    Eigen::Matrix4f& final_transform,
                    bool& hasICPConverged,
                    bool downsample = true);

            //rendered model aligns with input image
            void setSourceCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
            void getSourceCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud) const;

            //new frame aligns with previous frame
            void setTargetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
            void getTargetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) const;

            void setSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
            void getSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) const;

            void computeGlobalTransformation (Eigen::Matrix4f pair_transformation);

            Eigen::Affine3d getGlobalTransformation () const;

        private:
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source_cloud_with_normal_;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_;

            Eigen::Matrix4f global_transformation_;
    };
}
#endif
