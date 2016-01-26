#ifndef __OCTREE_H__
#define __OCTREE_H__

#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdint.h>

#include <pcl/console/print.h>
//#include <pcl/consloe/time.h>
#include <pcl/pcl_macros.h>
#include <boost/shared_ptr.hpp>

namespace MobileFusion {
    class OctreeNode {
        public:
            typedef boost::shared_ptr<OctreeNode> Ptr;
            typedef boost::shared_ptr<const OctreeNode> ConstPtr;
            OctreeNode ();
            OctreeNode (float x, float y, float z, float size_x, float size_y, float size_z);
            virtual ~OctreeNode ();
            void getCenter (float &x, float &y, float &z) const;
            void getSize (float &size_x, float &size_y, float &size_z) const;
            float getMaxSize () const;
            float getMinSize () const;
            bool hasChildren () const;
            std::vector<OctreeNode::Ptr>& getChildren ();
            const std::vector<OctreeNode::Ptr>& getChildren () const;
            void getLeaves (std::vector<OctreeNode::Ptr> &leaves, int num_levels);
            OctreeNode* getContainingVoxel (float x, float y, float z, float min_size=-1);
            const OctreeNode* getContainingVoxel (float x, float y, float z, float min_size=-1) const;
            bool getData (float &d, float&w) const;
            bool setData (float d, float w);
            virtual bool addObservation (float d_new, float w_new, float max_weight);
            virtual bool addObservation (float d_new, float w_new, float max_weight,
                                         uint8_t r, uint8_t g, uint8_t b);
            virtual bool getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const;
            virtual OctreeNode* instantiateNode (float x, float y, float z, float sx, float sy, float sz);
            virtual std::string getTypeString();
            static OctreeNode* instantiateByTypeString (const std::string &str);
            static OctreeNode* instantiateByTypeString (const std::string &str,
                                                        float x, float y, float z, float sx, float sy, float sz);
            void updateAverage ();
            std::vector<OctreeNode::Ptr>& split ();
            void splitRecursive (int num_left);
            float getVariance () const;
            //virtual void serialize (std::ostream &f) const;
            //virtual void deserialize (std::istream &f);

            float d_;
            float w_;
            float M_;
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
            RGBNode (float x, float y, float z, float size_x, float size_y, float size_z);
            RGBNode ();
            ~RGBNode ();
            virtual bool addObservation (float d_new, float w_new, float max_weight,
                                         uint8_t r, uint8_t g, uint8_t b);
            virtual bool getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const;
            virtual OctreeNode* instantiateNode (float x, float y, float z, float sx, float sy, float sz);
            virtual std::string getTypeString ();
            //virtual void serialize (std::ostream &f) const;
            //virtual void deserialize (std::istream &f);

        private:
            uint8_t r_;
            uint8_t g_;
            uint8_t b_;
    };

    class RGBNormalized : public OctreeNode {
        public:
            RGBNormalized (float x, float y, float z, float size_x, float size_y, float size_z);
            RGBNormalized ();
            virtual ~RGBNormalized();
            bool addObservation (float d_new, float w_new, float max_weight,
                                 uint8_t r, uint8_t g, uint8_t b);
            bool getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const;
            OctreeNode* instantiateNode (float x, float y, float z, float sx, float sy, float sz);
            std::string getTypeString ();
            //void serialize (std::ostream &f) const;
            //void deserialize (std::istream &f);

        private:
            float r_n_;
            float g_n_;
            float b_n_;
            float i_;
    };

    class Octree {
        public:
            typedef boost::shared_ptr<Octree> Ptr;
            typedef boost::shared_ptr<const Octree> ConstPtr;
            Octree (size_t res_x, size_t res_y, size_t res_z, float size_x, float size_y, float size_z, const std::string voxel_type = "NOCOLOR");
            Octree ();
            void init (int num_splits = 0);
            void init (float max_size_x, float max_size_y, float max_size_z);
            OctreeNode::Ptr& getRoot ();
            void getLeaves (std::vector<OctreeNode::Ptr> &leaves, int num_levels = -1) const;
            void getLeaves (std::vector<OctreeNode::Ptr> &leaves, float max_size_x, float max_size_y, float max_size_z) const;
            const OctreeNode* getContainingVoxel (float x, float y, float z, float min_size = -1) const;
            OctreeNode* getContainingVoxel (float x, float y, float z, float min_size = -1);
            //void serialize (std::ostream &f) const;
            //void deserialize (std::istream &f);
        protected:
            size_t res_x_, res_y_, res_z_;
            float size_x_, size_y_, size_z_;
            std::string voxel_type_;

            OctreeNode::Ptr root_;
            mutable std::vector<OctreeNode::Ptr> leaves_cached_;
    };
}

#endif
