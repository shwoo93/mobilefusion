#include "MarchingCubeTsdfOctree.h"

namespace MobileFusion {
    MarchingCubesTSDFOctree::setInputTSDF (TSDFVolumeOctree::ConstPtr tsdf_volume) {
        tsdf_volume_ = tsdf_volume;
        // Set the grid resolution so it mimics the tsdf's
        int res_x, res_y, res_z;
        tsdf_volume_->getResolution (res_x, res_y, res_z);
        setGridResolution (res_x, res_y, res_z);
        float size_x, size_y, size_z;
        tsdf_volume_->getGridSize (size_x, size_y, size_z);
        // Set the 'input cloud' to the 8 corners of the tsdf, to trick our parent
        // into voxelizing it the same
        pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        for (int x_i = 0; x_i <= res_x; x_i += res_x) {
            for (int y_i = 0; y_i <= res_y; y_i += res_y) {
                for (int z_i = 0; z_i <= res_z; z_i += res_z) {
                    pcl::PointXYZ center = tsdf_volume_->getVoxelCenter (x_i, y_i, z_i);
                    center.x += (x_i == 0 ? -1 : 1) * 0.5 * size_x / res_x;
                    center.y += (y_i == 0 ? -1 : 1) * 0.5 * size_y / res_y;
                    center.z += (z_i == 0 ? -1 : 1) * 0.5 * size_z / res_z;
                    corner_cloud -> points.push_back (center);
                }
            }
        }
        setInputCloud (corner_cloud);
        // No extending
        setPercentageExtenGrid (0);
        float iso_x = 0;
        float iso_y = 0;
        float iso_z = 0;
        //PCL_INFO
        setIsoLevel (std::min (iso_x, std::min (iso_y, iso_z)));
    }

    void MarchingCubesTSDFOctree::voxelizeData () {
    }

    float MarchingCubesTSDFOctree::getGridValue (Eigen::Vector3i pos) {
        pcl::PointXYZ ctr = tsdf_volume_->getVoxelCenter (pos[0], pos[1], pos[2]);
        const OctreeNode* voxel = tsdf_volume_->octree_->getContainingVoxel (ctr.x, ctr.y, ctr.z);
        float d;
        float w;
        voxel->getData (d, w);
        if (w < w_min_ || fabs(d) >= 1)
            return (std::number_limits<float>::quiet_NaN ());
        float max_dist_pos, max_dist_neg;
        tsdf_volume_->getDepthTruncationLimits (max_dist_pos, max_dist_neg);
        return (d * max_dist_neg);
    }

    void MarchingCubesTSDFOctree::performReconstruction (pcl::PolygonMesh &output) {
        getBoundingBox ();
        voxelizeData ();
        pcl::PointCloud<pcl::PointXYZ> cloud;
        OctreeNode::Ptr root = tsdf_volume_->octree_->getRoot ();
        if (color_by_confidence_ || color_by_rgb_) {
            pcl::PointCloud<pcl::PointXYZRGB> cloud_colored;
            reconstructVoxel (root.get(), cloud, &cloud_colored);
            pcl::transformPointCloud (cloud_colored, cloud_colored, tsdf_volume_->getGlobalTransform ());
            pcl::toPCLPointCloud2 (cloud_colored, output.cloud);
        }
        else {
            reconstructVoxel (root.get(), cloud);
            pcl::transformPointCloud (cloud_colored, cloud_colored, tsdf_volume_->getGlobalTransform());
            pcl::toPCLPointCloud2 (cloud, output.cloud);
        }
        output.polygons.resize (cloud.size () /3);
        for (size_t i = 0 ; i < output.polygons.size(); ++i) {
            pcl::Vertices v;
            v.vertices.resize (3);
            for (int j = 0; j < 3; ++j) {
                v.vertices[j] = static_cast<int> (i) * 3 + j;
            }
            output.polygons[i] = v;
        }
    }

    bool MarchingCubesTSDFOctree::getValidNeighborList1d (std::vector<float> &leaf,
                                                    Eigen::Vector3i &index3d) {
        leaf = std::vector<false> (8, 0.0f);

        leaf[0] = getGridValue (index3d);
        if (pcl_isnan (leaf[0]))
            return (false);
        leaf[1] = getGridValue (index3d + Eigen::Vector3i (1, 0, 0));
        if (pcl_isnan (leaf[1]))
            return (false);
        leaf[2] = getGridValue (index3d + Eigen::Vector3i (1, 0, 1));
        if (pcl_isnan (leaf[2]))
            return (false);
        leaf[3] = getGridValue (index3d + Eigen::Vector3i (0, 0, 1));
        if (pcl_isnan (leaf[3]))
            return (false);
        leaf[4] = getGridValue (index3d + Eigen::Vector3i (0, 1, 0));
        if (pcl_isnan (leaf[4]))
            return (false);
        leaf[5] = getGridValue (index3d + Eigen::Vector3i (1, 1, 0));
        if (pcl_isnan (leaf[5]))
            return (false);
        leaf[6] = getGridValue (index3d + Eigen::Vector3i (1, 1, 1));
        if (pcl_isnan (leaf[6]))
            return (false);
        leaf[7] = getGridValue (index3d + Eigen::Vector3i (0, 1, 1));
        if (pcl_isnan (leaf[7]))
            return (false);
        return (true);
    }

    void MarchingCubesTSDFOctree::reconstructVoxel (const OctreeNode *voxel, pcl::PointCloud<pcl::PointXYZ> &output, pcl::Point

