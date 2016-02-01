#ifndef __TSDF_VOLUME_OCTREE_H__
#define __TSDF_VOLUME_OCTREE_H__


#include <pcl/common/transforms.h>
#include <pcl/filters/frustum_culling.h>
#include <Eigen/Eigen>

#include "TsdfInterface.h"
#include "Octree.h"

namespace MobileFusion {
    class TSDFVolumeOctree : public TSDFInterface {
        public:
            typedef boost::shared_ptr<TSDFVolumeOctree> Ptr;
            typedef boost::shared_ptr<const TSDFVolumeOctree> ConstPtr;

            TSDFVolumeOctree ();
            ~TSDFVolumeOctree ();

            void setResolution (int xres, int yres, int zres);
            void getResolution (int& xres, int& yres, int& zres) const;

            void setGridSize (float xsize, float ysize, float zsize);
            void getGridSize (float& xsize, float& ysize, float& zsize) const;

            void setImageSize (int width, int height);
            void getImageSize (int& width, int& height) const;

            void setDepthTruncationLimits (float max_dist_pos, float max_dist_neg);
            void getDepthTruncationLimits (float& max_dist_pos, float& max_dist_neg) const;

            void setWeightTruncationLimit (float max_weight);
            float getWeightTruncationLimit () const;

            inline void setGlobalTransform (const Eigen::Affine3d &trans) {
                global_transform_ = trans; }

            inline Eigen::Affine3d getGlobalTransform () const {
                return (global_transform_); }

            void setCameraIntrinsics (const double focal_length_x,
                                      const double focal_length_y,
                                      const double principal_point_x,
                                      const double principal_point_y);

            void getCameraIntrinsics (double &focal_length_x,
                                      double &focal_length_y,
                                      double &principal_point_x,
                                      double &principal_point_y) const;

            inline void setMaxVoxelSize (float max_cell_size_x, float max_cell_size_y, float max_cell_size_z) {
                max_cell_size_x_ = max_cell_size_x;
                max_cell_size_y_ = max_cell_size_y;
                max_cell_size_z_ = max_cell_size_z;
            }

            inline void setIntegrateColor (bool integrate_color) {
                integrate_color_ = integrate_color; }

            inline void setNumRandomSplits (int num_random_splits) {
                num_random_splits_ = num_random_splits; }

            inline int getNumRandomSplits () {
                return (num_random_splits_);
            }

            inline void setSensorDistanceBounds (float min_sensor_dist, float max_sensor_dist) {
                min_sensor_dist_ = min_sensor_dist;
                max_sensor_dist_ = max_sensor_dist;
            }

            inline void getSensorDistanceBounds (float& min_sensor_dist, float& max_sensor_dist) const {
                min_sensor_dist = min_sensor_dist_;
                max_sensor_dist = max_sensor_dist_;
            }

            void reset ();

            template <typename PointT, typename NormalT> bool
            integrateCloud (const pcl::PointCloud<PointT>& cloud,
                            const pcl::PointCloud<NormalT>& normals,
                            const Eigen::Affine3d& trans = Eigen::Affine3d::Identity());

            pcl::PointCloud<pcl::PointNormal>::Ptr renderView (const Eigen::Affine3d &trans = Eigen::Affine3d::Identity(), int downsampleBy=1) const;
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr renderColoredView (const Eigen::Affine3d & trans = Eigen::Affine3d::Identity(), int downsampleBy=1) const;
            pcl::PointCloud<pcl::Intensity>::Ptr getIntensityCloud (const Eigen::Affine3d& trans = Eigen::Affine3d::Identity()) const;

            pcl::PointXYZ getVoxelCenter (size_t x, size_t y, size_t z) const;
            bool getVoxelIndex (float x, float y, float z, int& x_i, int& y_i, int& z_i) const;

            pcl::PointCloud<pcl::PointXYZ>::ConstPtr getVoxelCenters (int nlevels=4) const;

            void getFrustumCulledVoxels (const Eigen::Affine3d& trans, std::vector<OctreeNode::Ptr>& voxels) const;

            inline bool isEmpty() const {
                return is_empty_;
            }

            inline void setColorMode (const std::string& color_mode) {
                color_mode_ = color_mode;
            }

            void getOccupiedVoxelIndices (std::vector<Eigen::Vector3i> &indices) const;

            Octree::Ptr octree_;

            const float UNOBSERVED_VOXEL;
        protected:
            template <typename PointT, typename NormalT> int
            updateVoxel (const OctreeNode::Ptr& voxel,
                         const pcl::PointCloud<PointT>& cloud,
                         const pcl::PointCloud<NormalT>& normals,
                         const Eigen::Affine3f& trans);
            bool reprojectPoint (const pcl::PointXYZ &pt, int &u, int &v) const;
            float getTSDFValue (const Eigen::Vector3f& pt, bool* valid=NULL) const;
            float getTSDFValue (float x, float y, float z, bool* valid=NULL) const;
            float interpolateTrilinearly (const Eigen::Vector3f& pt, bool* valid=NULL) const;
            float interpolateTrilinearly (float x, float y, float z, bool* valid=NULL) const;
            bool getNeighbors (const pcl::PointXYZ& pt, std::vector<const OctreeNode*>& neighbors,
                               std::vector<pcl::PointXYZ>& centers) const;

            int xres_, yres_, zres_;

            float xsize_, ysize_, zsize_;

            float max_dist_pos_;
            float max_dist_neg_;

            float max_weight_;

            float min_sensor_dist_;
            float max_sensor_dist_;

            float max_cell_size_x_, max_cell_size_y_, max_cell_size_z_;

            int num_random_splits_;

            double focal_length_x_, focal_length_y_;
            double principal_point_x_, principal_point_y_;

            int image_width_, image_height_;

            bool is_empty_;

            bool integrate_color_;

            bool use_trilinear_interpolation_;

            Eigen::Affine3d global_transform_;

            std::string color_mode_;

            pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_centers_;
    };

    template <typename PointT, typename NormalT> bool
        TSDFVolumeOctree::integrateCloud (
                const pcl::PointCloud<PointT> &cloud,
                const pcl::PointCloud<NormalT> &normals,
                const Eigen::Affine3d &trans) {

            Eigen::Affine3f trans_inv = trans.inverse().cast<float> ();

            int px_step = 1;
            int nsplit = 0;
            for (size_t u = 0; u < cloud.width; u+= px_step) {
                for (size_t v = 0; v < cloud.height; v+= px_step) {

                    const PointT &pt_surface_orig = cloud (u, v);

                    if (pcl_isnan (pt_surface_orig.z))
                            continue;

                    int nstep = 0;
                    Eigen::Vector3f ray = pt_surface_orig.getVector3fMap ().normalized ();

                    for (int perm = 0; perm < num_random_splits_; perm++) {
                        PointT pt_trans;
                        float scale = (float)rand() / (float)RAND_MAX * 0.03;
                        Eigen::Vector3f noise = Eigen::Vector3f::Random ().normalized () * scale;
                        if (perm == 0 ) noise *= 0;
                        pt_trans.getVector3fMap () = trans.cast<float> () * (pt_surface_orig.getVector3fMap () + noise);
                        OctreeNode* voxel = octree_->getContainingVoxel (pt_trans.x, pt_trans.y, pt_trans.z);
                        if (voxel != NULL) {
                            while (voxel->getMinSize () > xsize_ / xres_) {
                                nsplit++;
                                voxel->split();
                                voxel = voxel->getContainingVoxel (pt_trans.x, pt_trans.y, pt_trans.z);
                            }
                        }
                    }
                }
            }

            std::vector<OctreeNode::Ptr> voxels_culled;
            getFrustumCulledVoxels(trans, voxels_culled);
        #pragma omp parallel for
            for (size_t i = 0; i < voxels_culled.size(); ++i) {
                updateVoxel (voxels_culled[i], cloud, normals, trans_inv);
            }

            is_empty_ = false;
            return (true);
    }

    inline float logNormal (float x, float mean, float var) {
        return (-std::pow (x - mean, 2) / (2*var));
    }

    template <typename PointT, typename NormalT> int
        TSDFVolumeOctree::updateVoxel (
                const OctreeNode::Ptr &voxel,
                const pcl::PointCloud<PointT> &cloud,
                const pcl::PointCloud<NormalT> &normals,
                const Eigen::Affine3f &trans_inv) {

            if (voxel->hasChildren ()) {
                std::vector<OctreeNode::Ptr>& children = voxel->getChildren();
                std::vector<bool> is_empty (children.size());
                for (size_t i = 0; i < children.size(); i++) {
                    is_empty[i] = updateVoxel (children[i], cloud, normals, trans_inv) < 0;
                }
                bool all_are_empty = true;
                for (size_t i = 0; i < is_empty.size(); i++)
                    all_are_empty &= is_empty[i];
                if (all_are_empty) {
                    children.clear ();
                }
                else {
                    return (1);
                }
            }

            pcl::PointXYZ v_g_orig;
            voxel->getCenter (v_g_orig.x, v_g_orig.y, v_g_orig.z);
            pcl::PointXYZ v_g = pcl::transformPoint (v_g_orig, trans_inv);
            if (v_g.z < min_sensor_dist_ || v_g.z > max_sensor_dist_)
                return (0);
            int u, v;
            if (!reprojectPoint (v_g, u, v))
                return (0);
            const PointT &pt = cloud (u,v);
            if (pcl_isnan (pt.z))
                return (0);
            float d;
            float w;
            voxel->getData (d, w);
            float d_new = (pt.z - v_g.z);
            if (fabs (d_new) < 3*voxel->getMaxSize()/4.) {
                float xsize, ysize, zsize;
                voxel->getSize (xsize, ysize, zsize);
                if (xsize > xsize_ / xres_ && ysize > ysize_ / yres_ && zsize > zsize_ / zres_) {
                    std::vector<OctreeNode::Ptr>& children = voxel->split();
                    std::vector<bool> is_empty (children.size());
                    for (size_t i = 0; i < children.size(); ++i) {
                        is_empty[i] = updateVoxel (children[i], cloud, normals, trans_inv) < 0;
                    }
                    bool all_are_empty = true;
                    for (size_t i = 0; i < is_empty.size (); ++i)
                        all_are_empty &= is_empty[i];
                    if (all_are_empty) {
                        children.clear();
                    }
                    else {
                        return (1);
                    }
                }
            }
            if (d_new > max_dist_pos_)
                d_new = max_dist_pos_;
            else if( d_new < -max_dist_neg_)
                return (0);

            d_new /= max_dist_neg_;

            float w_new = 1;

            if (integrate_color_)
                voxel->addObservation (d_new, w_new, max_weight_, pt.r, pt.g, pt.b);
            else
                voxel->addObservation (d_new, w_new, max_weight_);
            if (voxel->d_ < -0.99)
                return (0);
            else if (voxel->d_ < 0.99 * max_dist_pos_ / max_dist_neg_)
                return (1);
            else
                return (-1);
        }
}

#endif
