#ifndef __TSDF_INTERFACE_H__
#define __TSDF_INTERFACE_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <string>

namespace MobileFusion {
    class TSDFInterface {
        public:
            typedef boost::shared_ptr<TSDFInterface> Ptr;
            typedef boost::shared_ptr<const TSDFInterface> ConstPtr;

            virtual ~TSDFInterface() {}

            // res Number of voxels in the x direction
            virtual void setResolution (int xres, int yres, int zres) = 0;
            virtual void getResolution (int &xres, int &yres, int &zres) const = 0;
            // size extent of the grid's x direction, in meters
            virtual void setGridSize (float xsize, float ysize, float zsize) =0;
            virtual void getGridSize (float &xsize, float &ysize, float &zsize) const = 0;
            virtual void setDepthTruncationLimits (float max_dist_pos, float max_dist_neg) = 0;
            virtual void getDepthTruncationLimits (float &max_dist_pos, float &max_dist_neg) const = 0;
            virtual void setWeightTruncationLimit (float max_weight) = 0;
            virtual float getWeightTruncationLimit () const = 0;
            virtual void setGlobalTransform (const Eigen::Affine3d &trans) = 0;
            virtual Eigen::Affine3d getGlobalTransform () const = 0;
            virtual void setSensorDistanceBounds (float min_sensor_dist, float max_sensor_dist) const  = 0;
            //virtual void save (const std::string &filename) const = 0;
            //virtual void load (const std::string &filename) = 0;
            //static TSDFInterface::PTr instantiateFromFile (const std::string &filename);
            virtual bool getFxn (const pcl::PointXYZ &pt, float &val) const = 0;
            virtual bool getGradient (const pcl::PointXYZ &pt, Eigen::Vector3f &grad) const = 0;
            virtual bool getHessian (const pcl::PointXYZ &pt, Eigen::Matrix3f &hessian) const = 0;
            virtual bool getFxnAndGradient (const pcl::PointXYZ &pt, float &val, Eigen::Vector3f &grad) const {
                return (getFxn (pt, val) && getGradient (pt, grad));
            }
            virtual bool getFxnGradientAndHessian (const pcl::PointXYZ &pt,
                                                   float &val,
                                                   Eigen::Vector3f &grad,
                                                   Eigen::Matrix3f &hessian) const {
                return (getFxn (pt, val) && getGradient (pt, grad) && getHessian (pt, hessian));
            }
    };
}
#endif
