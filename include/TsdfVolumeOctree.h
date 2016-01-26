#ifndef __TSDF_VOLUME_OCTREE_H__
#define __TSDF_VOLUME_OCTREE_H__

#include <Eigen/Eigen>

#include "Octree.h"
#include "TsdfInterface.h"

namespace MobileFusion {
    class TSDFVolumeOctree : public TSDFInterface {
        public:
            typedef boost::shared_ptr<TSDFVolumeOctree> Ptr;
            typedef boost::shared_ptr<const TSDFVolumeOctree> ConstPtr;

            template <typename T> inline float sgn(T t) {
                return t > 0 ? 1 : t < 0 ? -1 : 0;
            }

            TSDFVolumeOctree ();
            ~TSDFVolumeOctree ();
            void setResolution (int xres, int yres, int zres);
            void getResolution (int &xres, int &yres, int &zres) const;
            void setGridSize (float xsize, float ysize, float zsize);
            void getGridSize (float &xsize, float &ysize, float &zsize) const;
            void setImageSize (int width, int height);
            void getImageSize (int &width, int &height) const;
            void setDepthTruncationLimits (float max_dist_pos, float max_dist_neg);
            void getDepthTruncationLimits (float &max_dist_pos, float &max_dist_neg) const;
            void setWeightTruncationLimit (float max_weight);
            float getWeightTruncationLimit () const;

            //inline void setGlobalTransform (const Eigen::Affine3d &trans) {
            //    global_transfrom_ = trans;
            //}

            void setGlobalTransform (const Eigen::Affine3d &trans);

            inline Eigen::Affine3d getGlobalTransform () const {
                return (global_transform_);
            }

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
                integrate_color_ = integrate_color;
            }

            inline void setNumRandomSplits (int num_random_splits) {
                num_random_splits_ = num_random_splits;
            }

            inline int getNumRandomSplits () {
                return num_random_splits_;
            }

            inline void setSensorDistanceBounds (float min_sensor_dist, float max_sensor_dist) const {
                min_sensor_dist = min_sensor_dist_;
                max_sensor_dist = max_sensor_dist_;
            }

            void reset ();
            //void save (const std::string &filename) const;
            //void load (const std::string &filename);
            bool getFxn (const pcl::PointXYZ &pt, float &val);
            bool getGradient (const pcl::PointXYZ &pt, Eigen::Vector3f &grad);
            bool getHessian (const pcl::PointXYZ &pt, Eigen::Matrix3f &hessian);
            bool getFxnAndGradient (const pcl::PointXYZ &pt, float &val ,Eigen::Vector3f &grad);
            bool getFxnGradientAndHessian (const pcl::PointXYZ &pt,
                                       float &val,
                                       Eigen::Vector3f &grad,
                                       Eigen::Matrix3f &hessian);
            template <typename PointT, typename NormalT> bool
                integrateCloud (const pcl::PointCloud<PointT> &cloud,
                                const pcl::PointCloud<NormalT> &normals,
                                const Eigen::Affine3d &trans = Eigen::Affine3d::Identity());
            pcl::PointCloud<pcl::PointNormal>::Ptr
                renderView (const Eigen::Affine3d &trans = Eigen::Affine3d::Identity(),
                            int downsampleBy=1) const;
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
                renderColorView (const Eigen::Affine3d &trans = Eigen::Affine3d::Identity(),
                                 int downsampleBy=1) const;
            pcl::PointCloud<pcl::Intensity>::Ptr
                getIntensityCloud (const Eigen::Affine3d &trans = Eigen::Affine3d::Identity()) const;

            pcl::PointXYZ getVoxelCenter (size_t x, size_t y, size_t z) const;
            bool getVoxelIndex (float x, float y, float z, int &x_i, int &y_i, int &z_i) const;
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr getVoxelCenters (int nlevels=4) const;
            void getFrustumCulledVoxels (const Eigen::Affine3d &trans, std::vector<OctreeNode::Ptr> &voxels) const;

            inline bool isEmpty() {
                return is_empty_;
            }

            inline void setColorMode (const std::string &color_mode) {
                color_mode_ = color_mode;
            }
            void getOccupiedVoxelIndices (std::vector<Eigen::Vector3i> &indices) const;

            Octree::Ptr octree_;
            const float UNOBSERVED_VOXEL;

        protected:
            template <typename PointT, typename NormalT> int
                updateVoxel (const OctreeNode::Ptr &voxel,
                             const pcl::PointCloud<PointT> &cloud,
                             const pcl::PointCloud<NormalT> &normals,
                             const Eigen::Affine3f &trans);
            bool reprojectPoint (const pcl::PointXYZ &pt, int &u, int &v) const;
            float getTSDFValue (const Eigen::Vector3f &pt, bool* valid = NULL) const;
            float getTSDFValue (float x, float y, float z, bool * valid = NULL) const;
            float interpolateTrilinearly (const Eigen::Vector3f &pt, bool* valid = NULL) const;
            float interpolateTrilinearly (float x, float y, float z, bool* valid = NULL) const;
            bool getNeighbors (const pcl::PointXYZ &pt, std::vector<const OctreeNode*> &neighbors,
                               std::vector<pcl::PointXYZ> &centers) const;

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
            bool weight_by_depth_;
            bool weight_by_variance_;
            bool integrate_color_;
            bool use_trilinear_interpolation_;
            Eigen::Affine3d global_transform_;
            std::string color_mode_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_centers_;
    };
}

#include "TsdfVolumeOctree.hpp"
#endif
