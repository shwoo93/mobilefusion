#include "CpuTsdf.h"

namespace MobileFusion {
    CpuTsdf::CpuTsdf()
    : tsdf_(new cpu_tsdf::TSDFVolumeOctree)
    , octree_()
    , mesh_()
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

    void CpuTsdf::constructMesh() {
        octree_.setInputTSDF(tsdf_);
        octree_.reconstruct(mesh_);

        vis_->addPolygonMesh(mesh_);
        vis_->spin();
    }
}
