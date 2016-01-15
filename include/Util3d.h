#ifndef __UTIL3D_H__
#define __UTIL3D_H__

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/core/core.hpp>

namespace MobileFusion {
    namespace util3d {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromRgbd(
                const cv::Mat &rgb,
                const cv::Mat &depth,
                float cx, float cy,
                float fx, float fy,
                int decimation);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDepth(
                const cv::Mat &depth,
                float cx, float cy,
                float fx, float fy,
                int decimation);
        pcl::PointXYZ projectDepthTo3D(
                const cv::Mat &depth,
                float x, float y,
                float cx, float cy,
                float fx, float fy);
        void rgbdFromCloud(
                const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                float &cx, float &cy, float &fx, float &fy,
                cv::Mat &depth, cv::Mat &rgb);
    }
}

#endif
