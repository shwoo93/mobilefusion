#include "CpuTsdf.h"

namespace MobileFusion {
    CpuTsdf::CpuTsdf()
    : tsdf_(new cpu_tsdf::TSDFVolumeOctree)
    , octree_()
    , mesh_() {
        tsdf_->setGridSize(1., 1., 1.);
        tsdf_->setResolution(128, 128, 128);
        tsdf_->setIntegrateColor(true);
        tsdf_->reset();
        octree_.setMinWeight(2);
        octree_.setColorByRGB(true);
    }

    CpuTsdf::~CpuTsdf() {
    }

    void CpuTsdf::integrateCloud(
            const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
            const pcl::PointCloud<pcl::PointXYZRGB> &normals,
            const Eigen::Affine3d &trans) {
        tsdf_->integrateCloud(cloud,normals,trans);
    }

    void CpuTsdf::constructMesh() {
        octree_.setInputTSDF(tsdf_);
        octree_.reconstruct(mesh_);
    }
}
