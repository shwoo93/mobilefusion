#include "CpuTsdf.h"

#include "boost/format.hpp"

namespace MobileFusion {
    CpuTsdf::CpuTsdf()
    : tsdf_(new cpu_tsdf::TSDFVolumeOctree)
    , octree_()
    , vis_(new pcl::visualization::PCLVisualizer)
    , mesh_count_(0) {
        tsdf_->setGridSize(3., 3., 3.);
        tsdf_->setResolution(512, 512, 512);
        tsdf_->setIntegrateColor(true);
        tsdf_->reset();
        tsdf_->setImageSize(512, 424);
        tsdf_->setCameraIntrinsics(540.686f, 540.686f, 256.0f, 212.0f);
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

    pcl::PointCloud<pcl::PointNormal>::Ptr
        CpuTsdf::renderView(const Eigen::Affine3d &trans, int downsampleBy) {

        return tsdf_->renderView (trans, downsampleBy);
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr CpuTsdf::renderColoredView(
            const Eigen::Affine3d &trans, int downsampleBy) {

        return tsdf_->renderColoredView (trans, downsampleBy);
    }


    void CpuTsdf::constructMesh() {
        mesh_count_++;
        octree_.setInputTSDF(tsdf_);
        pcl::PolygonMesh mesh;
        octree_.reconstruct(mesh);
        std::string mesh_name = str(boost::format("/home/vllab/Desktop/mesh/test%1%") % mesh_count_);
        pcl::io::savePolygonFilePLY(mesh_name, mesh);
    }
}
