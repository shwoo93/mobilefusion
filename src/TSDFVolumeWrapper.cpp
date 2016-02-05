#include "TSDFVolumeWrapper.h"

#include "boost/format.hpp"

namespace MobileFusion {
    TSDFVolumeWrapper::TSDFVolumeWrapper()
    : tsdf_volume_(new cpu_tsdf::TSDFVolumeOctree)
    , mc_() {
    }

    TSDFVolumeWrapper::~TSDFVolumeWrapper() {
    }

    void TSDFVolumeWrapper::integrateCloud(
            const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
            const pcl::PointCloud<pcl::Normal>& normals,
            const Eigen::Affine3d& trans) {
        assert(!cloud.empty());
        tsdf_volume_->integrateCloud(cloud, normals, trans);
    }

    void TSDFVolumeWrapper::setGridSize (float xsize, float ysize, float zsize) {
        tsdf_volume_->setGridSize(xsize, ysize, zsize);
    }

    void TSDFVolumeWrapper::setResolution (int xres, int yres, int zres) {
        tsdf_volume_->setResolution(xres, yres, zres);
    }

    void TSDFVolumeWrapper::setIntegrateColor (bool integrate_color) {
        tsdf_volume_->setIntegrateColor(integrate_color);
    }

    void TSDFVolumeWrapper::reset() {
        tsdf_volume_->reset();
    }

    void TSDFVolumeWrapper::setImageSize (int width, int height) {
        tsdf_volume_->setImageSize(width, height);
    }

    void TSDFVolumeWrapper::setCameraIntrinsics (double focal_length_x,
                                                 double focal_length_y,
                                                 double principal_point_x,
                                                 double principal_point_y) {

        tsdf_volume_->setCameraIntrinsics (focal_length_x,
                                           focal_length_y,
                                           principal_point_x,
                                           principal_point_y);
    }

    void TSDFVolumeWrapper::setMinWeight (float w_min) {
        mc_.setMinWeight (w_min);
    }

    void TSDFVolumeWrapper::setColorByRGB (bool color_by_rgb) {
        mc_.setColorByRGB (color_by_rgb);
    }

    void TSDFVolumeWrapper::setInputTSDF () {
        mc_.setInputTSDF (tsdf_volume_);
    }

    void TSDFVolumeWrapper::reconstruct (pcl::PolygonMesh& mesh) {
        mc_.reconstruct (mesh);
    }
}
