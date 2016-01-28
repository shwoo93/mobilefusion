#include "CpuTsdf.h"

namespace MobileFusion {
    CpuTsdf::CpuTsdf()
    : tsdf_(new cpu_tsdf::TSDFVolumeOctree)
    , octree_()
    , vis_(new pcl::visualization::PCLVisualizer) {
        tsdf_->setGridSize(1., 1., 1.);
        tsdf_->setResolution(128, 128, 128);
        tsdf_->setImageSize(512, 424);
        tsdf_->setCameraIntrinsics(540.686f, 540.686f, 256.0f, 212.0f);
        tsdf_->setIntegrateColor(true);
        tsdf_->setWeightTruncationLimit(50.f);
        tsdf_->reset();
        octree_.setMinWeight(2);
        octree_.setColorByRGB(true);
    }

    CpuTsdf::~CpuTsdf() {
    }

    void CpuTsdf::integrateCloud(
            const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
            const pcl::PointCloud<pcl::Normal> &normals,
            const Eigen::Affine3d &trans) {
        tsdf_->integrateCloud(cloud, normals, trans);
    }
    pcl::PointCloud<pcl::PointNormal>::Ptr CpuTsdf::renderView(const Eigen::Affine3d &trans, int downsampleBy) {
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
        cloud = tsdf_->renderView (trans, downsampleBy);
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr CpuTsdf::renderColoredView(
            const Eigen::Affine3d &trans, int downsampleBy) {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        cloud = tsdf_->renderColoredView (trans, downsampleBy);
        return cloud;
    }


    void CpuTsdf::constructMesh() {
        octree_.setInputTSDF(tsdf_);
        pcl::PolygonMesh mesh;
        octree_.reconstruct(mesh);
        pcl::io::savePolygonFilePLY("test.ply",mesh);
    }
}
