#include "util3d_registration.h"

#include <pcl/registration/gicp.h>
#include <pcl/registration/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>


namespace mobilefusion {
    namespace util3d {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(
                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                const Eigen::Matrix4f &transform) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*cloud, *output, transform);
            return output;
        }

        Eigen::Matrix4f transformationIcp_Xyzrgb(
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_source,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_target) {

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sourceDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_targetDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);


            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud_source, *cloud_source, indices);
            pcl::removeNaNFromPointCloud(*cloud_target, *cloud_target, indices);


            pcl::VoxelGrid<pcl::PointXYZRGB> filter;
            filter.setInputCloud(cloud_source);
            filter.setLeafSize(0.1f, 0.1f, 0.1f);
            filter.filter(*cloud_sourceDownsampled);

            filter.setInputCloud(cloud_target);
            filter.setLeafSize(0.1f, 0.1f, 0.1f);
            filter.filter(*cloud_targetDownsampled);

            std::cout << "sampled number of source point cloud : " << cloud_sourceDownsampled->size() << std::endl;
            std::cout << "sampled number of target point cloud : " << cloud_targetDownsampled->size() << std::endl;


            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

            icp.setInputTarget(cloud_targetDownsampled);
            icp.setInputSource(cloud_sourceDownsampled);


            pcl::PointCloud<pcl::PointXYZRGB> cloud_source_registered;
            icp.align(cloud_source_registered);


            if (!icp.hasConverged())
                std::cout << "Gicp does not converged! you need to reset parameters of gicp!" << std::endl;

            return icp.getFinalTransformation();
        }

        Eigen::Matrix4f transformation_Xyz(
                pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target)
        {
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sourceDownsampled(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_targetDownsampled(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::VoxelGrid<pcl::PointXYZ> filter;
            filter.setLeafSize(0.1f, 0.1f, 0.1f);
            filter.setInputCloud(cloud_source);
            filter.filter(*cloud_sourceDownsampled);

            filter.setLeafSize(0.1f, 0.1f, 0.1f);
            filter.setInputCloud(cloud_target);
            filter.filter(*cloud_targetDownsampled);

            icp.setInputTarget(cloud_targetDownsampled);
            icp.setInputTarget(cloud_sourceDownsampled);

            pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
            icp.align(cloud_source_registered);

            return icp.getFinalTransformation();

        }
    }
}
