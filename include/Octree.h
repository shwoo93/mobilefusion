#ifndef __OCTREE_H__
#define __OCTREE_H__

#include "OctreeNode.h"

namespace MobileFusion {
    class Octree {
        public:
            typedef boost::shared_ptr<Octree> Ptr;
            typedef boost::shared_ptr<const Octree> ConstPtr;

            Octree (size_t res_x, size_t res_y, size_t res_z, float size_x, float size_y, float size_z, const std::string voxel_type = "NOCOLOR");
            Octree () {}

            void init (int num_splits=0);
            void init (float max_size_x, float max_size_y, float max_size_z);

            OctreeNode::Ptr& getRoot ();

            void getLeaves (std::vector<OctreeNode::Ptr>& leaves, int num_levels = -1) const;
            void getLeaves (std::vector<OctreeNode::Ptr>& leaves, float max_size_x, float max_size_y, float max_size_z) const;

            const OctreeNode* getContainingVoxel (float x, float y, float z, float min_size=-1) const;
            OctreeNode* getContainingVoxel (float x, float y, float z, float min_size=-1);

        protected:
            size_t res_x_, res_y_, res_z_;
            float size_x_, size_y_, size_z_;
            std::string voxel_type_;

            OctreeNode::Ptr root_;
            mutable std::vector<OctreeNode::Ptr> leaves_cached_;
    };
}

#endif
