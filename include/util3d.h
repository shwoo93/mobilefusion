#ifndef UTIL3D_H_
#define UTIL3D_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/core/core.hpp>

namespace mobilefusion {
namespace util3d {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromRgbd(
            const cv::Mat &rgb,
            const cv::Mat &depth,
            float cx, float cy,
            float fx, float fy,
            int decimation);

    void rgbdFromCloud(
            const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
            float &cx, float &cy, float &fx, float &fy,
            cv::Mat &depth, cv::Mat &rgb);
}
}

#endif
