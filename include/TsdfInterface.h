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

            virtual ~TSDFInterface () {}

            virtual void setResolution (int xres, int yres, int zres) = 0;
            virtual void getResolution (int& xres, int& yres, int& zres) = 0;

            virtual void setGridSize (float xsize, float ysize, float zsize) = 0;
            virtual void getGridSize (float& xsize, float& ysize, float& zsize) = 0;

            virtual void setDepthTruncationLimits (float max_dist_pos, float max_dist_neg) = 0;
            virtual void getDepthTruncationLimits (float& max_dist_pos, float max_dist_neg) const = 0;

            virtual void setWeightTruncationLimit (float max_weight) = 0;
            virtual float getWeightTruncationLimit () const = 0;

            virtual void setGlobalTransform (const Eigen::Affine3d &trans) = 0;
            virtual Eigen::Affine3d getGlobalTransform () = 0;

            virtual void setSensorDistanceBounds (float min_sensor_dist, float max_sensor_dist) = 0;
            virtual void getSensorDistanceBounds (float& min_sensor_dist, float& max_sensor_dist) const = 0;
    };
}

#endif
