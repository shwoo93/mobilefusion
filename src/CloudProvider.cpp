#include "CloudProvider.h"

#include <limits>

#include "boost/format.hpp"

#include "CloudListener.h"

namespace MobileFusion{
    CloudProvider::CloudProvider()
    : cx_(256.0f)
    , cy_(212.0f)
    , fx_(540.686f)
    , fy_(540.686f)
    , cloud_listeners_() {
    }

    CloudProvider::~CloudProvider() {
    }

    void CloudProvider::onFrame(const cv::Mat& rgb, const cv::Mat &depth) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (cloudFromRgbd(rgb, depth));

        for(std::vector<boost::shared_ptr<CloudListener> >::iterator iter = cloud_listeners_.begin() ; iter != cloud_listeners_.end() ; iter++) {
            (*iter)->onCloudFrame(cloud);
        }
    }

    void CloudProvider::addListener(boost::shared_ptr<CloudListener> cloud_listener) {
        cloud_listeners_.push_back(cloud_listener);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudProvider::cloudFromRgbd(const cv::Mat& rgb, const cv::Mat& depth) {
        assert(rgb.rows == depth.rows && rgb.cols == depth.cols);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        cloud->width = rgb.cols;
        cloud->height = rgb.rows;
        cloud->is_dense = false;
        cloud->resize(cloud->height * cloud->width);

        for(int i = 0 ; i < cloud->width; ++i) {
            for(int j = 0 ; j < cloud->height; ++j) {
                pcl::PointXYZRGB& pt = cloud->at(i, j);

                cv::Vec3b v(rgb.at<cv::Vec3b>(j, i));
                pt.b = v[0];
                pt.g = v[1];
                pt.r = v[2];

                //depth in meters
                if(depth.at<float>(j, i) == 0.0f) {
                    pt.z = std::numeric_limits<float>::quiet_NaN();
                }
                else {
                    pt.z = depth.at<float>(j, i) * 0.001f;
                }
                pt.x = (static_cast<float>(i) - cx_) * pt.z / fx_;
                pt.y = (static_cast<float>(j) - cy_) * pt.z / fy_;
            }
        }

        return cloud;
    }
}

