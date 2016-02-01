#ifndef __OCTREE_NODE_H__
#define __OCTREE_NODE_H__

#pragma once
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/pcl_macros.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <stdint.h>
#include <cmath>
#include <fstream>
#include <iostream>

namespace MobileFusion {
    class OctreeNode {
        public:
            typedef boost::shared_ptr<OctreeNode> Ptr;
            typedef boost::shared_ptr<const OctreeNode> ConstPtr;

            OctreeNode () {}
            OctreeNode (float x, float y, float z, float size_x, float size_y, float size_z);
            virtual ~OctreeNode () {}

            void getCenter (float& x, float& y, float& z) const;
            void getSize (float& size_x, float& size_y, float& size_z) const;

            float getMaxSize () const;
            float getMinSize () const;

            bool hasChildren () const;
            std::vector<OctreeNode::Ptr>& getChildren();
            const std::vector<OctreeNode::Ptr>& getChildren() const;
            void getLeaves (std::vector<OctreeNode::Ptr>& leaves, int num_levels);

            OctreeNode* getContainingVoxel (float x, float y, float z, float min_size = -1);
            const OctreeNode* getContainingVoxel (float x, float y, float z, float min_size = -1) const;

            bool getData (float& d, float& w) const;
            bool setData (float d, float w);

            virtual bool addObservation (float d_new, float w_new, float max_weight);
            virtual bool addObservation (float d_new ,float w_new, float max_weight,
                                         uint8_t r, uint8_t g, uint8_t b);

            virtual bool getRGB (uint8_t& r, uint8_t& g, uint8_t& b) const;

            virtual OctreeNode* instantiateNode (float x, float y, float z, float sx, float sy, float sz);

            virtual std::string getTypeString ();
            static OctreeNode* instantiateByTypeString (const std::string &str);
            static OctreeNode* instantiateByTypeString (const std::string &str,
                                                        float x, float y, float z, float sx, float sy, float sz);

            void updateAverage();

            std::vector<OctreeNode::Ptr>& split();
            void splitRecursive (int num_left);

            float d_;
            float w_;
            int nsample_;
        protected:
            float ctr_x_;
            float ctr_y_;
            float ctr_z_;
            float size_;
            std::vector<OctreeNode::Ptr> children_;
    };

    class RGBNode : public OctreeNode {
        public:
            RGBNode (float x, float y, float z, float size_x, float size_y, float size_z):
                r_(0),
                g_(0),
                b_(0),
                OctreeNode (x, y, z, size_x, size_y, size_z) {}
            RGBNode () :
                OctreeNode ()
            {}

            virtual ~RGBNode () {}

            virtual bool addObservation (float d_new, float w_new, float max_weight,
                                         uint8_t r, uint8_t g, uint8_t b);

            virtual bool getRGB (uint8_t& r, uint8_t&g, uint8_t& b) const;

            virtual OctreeNode* instantiateNode (float x, float y, float z, float sx, float sy, float sz);

            uint8_t r_;
            uint8_t g_;
            uint8_t b_;
    };

}
#endif
