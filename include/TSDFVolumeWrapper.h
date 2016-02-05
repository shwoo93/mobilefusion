#ifndef __TSDF_VOLUME_WRAPPER_H__
#define __TSDF_VOLUME_WRAPPER_H__

#include <vector>

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/octree.h>
#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <eigen_extensions/eigen_extensions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

namespace MobileFusion {

    class TSDFVolumeWrapper {
        public:
            TSDFVolumeWrapper();

            ~TSDFVolumeWrapper();

            void integrateCloud (
                    const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                    const pcl::PointCloud<pcl::Normal>& normals,
                    const Eigen::Affine3d &trans = Eigen::Affine3d::Identity());

            void setGridSize (float xsize, float ysize, float zsize);

            void setResolution (int xres, int yres, int zres);

            void setIntegrateColor (bool integrate_color);

            void reset ();

            void setImageSize (int width, int height);

            void setCameraIntrinsics (const double focal_length_x,
                                      const double focal_length_y,
                                      const double principal_point_x,
                                      const double principal_point_y);

            void setMinWeight (float w_min);

            void setColorByRGB (bool color_by_rgb);

            void setInputTSDF ();

            void reconstruct (pcl::PolygonMesh& mesh);

        private:
            cpu_tsdf::TSDFVolumeOctree::Ptr tsdf_volume_;
            cpu_tsdf::MarchingCubesTSDFOctree mc_;
    };
}

#endif
