#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "boost/format.hpp"

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointXYZRGBNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

pcl::visualization::PCLVisualizer *p;
int vp_1 = 1;
int vp_2 = 2;

void showCloudsLeft (const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source) {
    p->removePointCloud ("vp1_target");
    p->removePointCloud ("vp1_source");

    PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
    p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
    p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

    PCL_INFO ("Press q to begin the registration.\n");
    p->spin();
}

void showCloudsRight (const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source) {
    p->removePointCloud ("source");
    p->removePointCloud ("target");

    PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
    PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");

    p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
    p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

    p->spinOnce();
}

void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downsample = false) {

    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample) {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);

        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else {
        src = cloud_src;
        tgt = cloud_tgt;
    }

    //compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    //align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-6);
    reg.setMaxCorrespondenceDistance (0.1);

    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);

    //run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (2);

    for (int i = 0; i < 30 ; ++i) {

        points_with_normals_src = reg_result;

        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);

        Ti = reg.getFinalTransformation() * Ti;

        if (fabs ((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

        prev = reg.getLastIncrementalTransformation ();

        showCloudsRight (points_with_normals_tgt, points_with_normals_src);
    }

    targetToSource = Ti.inverse();

    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

    p->removePointCloud ("source");
    p->removePointCloud ("target");

    PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
    p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
    p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

    PCL_INFO ("press q to continue the registration.\n");

    p->spin();

    p->removePointCloud ("source");
    p->removePointCloud ("target");

    *output += *cloud_src;

    final_transform = targetToSource;
}

int main (int argc, char** argv) {

    pcl::PCDReader reader;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> data;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    int index = 1;

    while(true) {

        std::cout<<"# iteration: "<< index << std::endl;
        std::string cloud_name = str(boost::format ("/home/vllab/Desktop/PCDfiles/cloud%1%.pcd") % index);

        if (index == 50)
            break;

        if (reader.read(cloud_name, *cloud) == -1) {
            std::cout<<"no PCDfile"<<std::endl;
            break;
        }

        data.push_back(cloud);
        ++index;
    }

    if (data.empty()) {
        std::cout<<"here1"<<std::endl;
        return -1;
    }

    p = new pcl::visualization::PCLVisualizer ("pairwise incremental registration example");
    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

    PointCloud::Ptr result (new PointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

    for (size_t i = 1; i < data.size(); ++i) {

        std::cout<<i<<std::endl;

        source = data[i-1];
        target = data[i];

        showCloudsLeft (source, target);
        std::cout<<"here2"<<std::endl;

        PointCloud::Ptr temp (new PointCloud);
        pairAlign (source, target, temp, pairTransform, true);
        pcl::transformPointCloud (*temp, *result, GlobalTransform);
        GlobalTransform = GlobalTransform * pairTransform;
        std::string result_name = str(boost::format ("/home/vllab/Desktop/ICPResult/result%1%.pcd") % i);
        pcl::io::savePCDFile (result_name, *result, true);
    }
}

