#ifndef __CPU_TSDF_H__
#define __CPU_TSDF_H__

#include <vector>

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/octree.h>
#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <eigen_extensions/eigen_extensions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>


namespace MobileFusion {
    class CpuTsdf {
        public:
            CpuTsdf();
            ~CpuTsdf();
            void integrateCloud(
                    const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                    const pcl::PointCloud<pcl::PointXYZRGB> &normals,
                    const Eigen::Affine3d &trnas);
            void constructMesh();
        private:
            cpu_tsdf::TSDFVolumeOctree::Ptr tsdf_;
            cpu_tsdf::MarchingCubesTSDFOctree octree_;
            pcl::PolygonMesh mesh_;
    };
}



#endif
