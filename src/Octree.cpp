#include "Octree.h"

namespace MobileFusion {
    Octree::Octree (size_t res_x, size_t res_y, size_t res_z, float size_x, float size_y, float size_z, const std::string voxel_type)
    : res_x_ (res_x)
    , res_y_ (res_y)
    , res_z_ (res_z)
    , size_x_ (size_x)
    , size_y_ (size_y)
    , size_z_ (size_z)
    , voxel_type_ (voxel_type)
    {}


    void Octree::init (int num_splits) {
        root_.reset (OctreeNode::instantiateByTypeString (voxel_type_, 0, 0, 0, size_x_, size_y_, size_z_));
        root_->splitRecursive(num_splits);
    }

    void Octree::init (float max_size_x, float max_size_y, float max_size_z) {
        int desired_res = std::max (size_x_/max_size_x, std::max (size_y_ / max_size_y, size_z_/max_size_z));
        int num_levels = std::ceil (std::log (desired_res) / std::log (2));
        init (num_levels);
    }

    OctreeNode::Ptr & Octree::getRoot() {
        return root_;
    }

    void Octree::getLeaves (std::vector<OctreeNode::Ptr>& leaves, int num_levels) const {
        if (num_levels == 0)
            leaves.push_back (root_);
        else
            root_->getLeaves (leaves, num_levels-1);
    }

    void Octree::getLeaves (std::vector<OctreeNode::Ptr>& leaves, float max_size_x, float max_size_y, float max_size_z) const {
        int desired_res = std::max (size_x_/max_size_x, std::max (size_y_/max_size_y, size_z_/max_size_z));
        int num_levels = std::ceil (std::log (desired_res) / std::log (2));
        getLeaves (leaves, num_levels);
    }

    const OctreeNode* Octree::getContainingVoxel (float x, float y, float z, float min_size) const {
        if (pcl_isnan (z) || std::fabs (x) > size_x_/2 || std::fabs (y) > size_y_/2 || std::fabs (z) > size_z_/2)
            return (NULL);
        return (root_->getContainingVoxel (x, y, z, min_size));
    }

    OctreeNode* Octree::getContainingVoxel (float x, float y, float z, float min_size) {
        if (pcl_isnan (z) || std::fabs (x) > size_x_/2 || std::fabs (y) > size_y_/2 || std::fabs (z) > size_z_/2)
            return (NULL);
        return (root_->getContainingVoxel (x, y, z, min_size));
    }
}
