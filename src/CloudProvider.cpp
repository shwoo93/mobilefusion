#include "CloudProvider.h"

#include "CloudListener.h"

namespace MobileFusion{
    CloudProvider::CloudProvider()
    : cx_(256.0f)
    , cy_(212.0f)
    , fx_(540.686f)
    , fy_(540.686f)
    , decimation_(5)
    , cloud_listeners_() {
    }

    CloudProvider::~CloudProvider() {
    }

    void CloudProvider::onFrame(cv::Mat &rgb, cv::Mat &depth) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(cloudFromRgbd(rgb, depth));
        for(std::vector<boost::shared_ptr<CloudListener> >::iterator iter = cloud_listeners_.begin() ; iter != cloud_listeners_.end() ; iter++) {
            (*iter)->onCloudFrame(cloud);
        }
    }

    void CloudProvider::addListener(boost::shared_ptr<CloudListener> cloud_listener) {
        cloud_listeners_.push_back(cloud_listener);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudProvider::cloudFromRgbd(const cv::Mat& rgb, const cv::Mat& depth) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        if(decimation_ < 1) {
            return cloud;
        }

        cloud->height = depth.rows / decimation_;
        cloud->width = depth.cols / decimation_;
        cloud->is_dense = false;
        cloud->resize(cloud->height * cloud->width);

        for(int h = 0 ; h < depth.rows && h / decimation_ < static_cast<int>(cloud->height); h += decimation_) {
            for(int w = 0 ; w < depth.cols && w / decimation_ < static_cast<int>(cloud->width); w += decimation_) {

                pcl::PointXYZRGB& pt = cloud->at((h / decimation_) * cloud->width + (w / decimation_));

                pt.b = rgb.at<cv::Vec3b>(h, w)[0];
                pt.g = rgb.at<cv::Vec3b>(h, w)[1];
                pt.r = rgb.at<cv::Vec3b>(h, w)[2];

                //float depth_point = static_cast<float>(depth.at<unsigned short>(h,w)) * 0.001f;
                float depth_point = depth.at<float>(h,w);
                pt.x = (static_cast<float>(w) - cx_) * depth_point / fx_;
                pt.y = (static_cast<float>(h) - cy_) * depth_point / fy_;
                pt.z = depth_point;
            }
        }

        return cloud;
    }
}

