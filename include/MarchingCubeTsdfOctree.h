#ifndef __MARCHING_CUBE_TSDF_OCTREE_H__
#define __MARCHING_CUBE_TSDF_OCTREE_H__

#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/impl/marching_cubes.hpp>

#include "TsdfVolumeOctree.h"

namespace MobileFusion {
    class MarchingCubesTSDFOctree :public pcl::MarchingCubes<pcl::PointXYZ> {
        using pcl::MarchingCubes<pcl::PointXYZ>::grid_;

        public:
            MarchingCubesTSDFOctree();
            void setInputTSDF (TSDFVolumeOctree::ConstPtr tsdf_volume);
            bool getValidNeighborList1D (std::vector<float> &leaf,
                                         Eigen::Vector3i &index3d);
            inline void setColorByConfidnece (bool color_by_confidence) {
                color_by_confidence_ = color_by_confidence;
            }
            inline void setColorByRGB (bool color_by_rgb) {
                color_by_rgb_ = color_by_rgb;
            }
            inline void setMinWeight (float w_min) {
                w_min_ = w_min;
            }
        protected:
            void voxelizeData ();
            float getGridValue (Eigen::Vector3i pos);
            void perfomReconstruction (pcl::PolygonMesh &output);
            void
            reconstructVoxel (const OctreeNode *voxel,
                    pcl::PointCloud<pcl::PointXYZ> &output,
                    pcl::PointCloud<pcl::PointXYZRGB> *output_colored=NULL);
            TSDFVolumeOctree::ConstPtr tsdf_volume_;
            bool color_by_confidence_;
            bool color_by_rgb_;
            float w_min_;
    };
}
#endif
