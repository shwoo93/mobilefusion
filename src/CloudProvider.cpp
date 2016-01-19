#include "CloudProvider.h"

namespace MobileFusion{
    CloudProvider::CloudProvider()
    : cx_(256.0f)
    , cy_(212.0f)
    , fx_(540.686f)
    , fy_(540.686f)
    , decimation_(5) {
    }

    CloudProvider::~CloudProvider() {
    }

    void CloudProvider::setCameraIntrinsic(float cx, float cy, float fx, float fy) {
        cx_ = cx;
        cy_ = cy;
        fx_ = fx;
        fy_ = fy;
    }

    void CloudProvider::setDecimation(int decimation) {
        decimation_ = decimation;
    }

    void CloudProvider::OnFrame(cv::Mat &rgb, cv::Mat &depth) {
        cloud_ = MobileFusion::FrameToCloud::cloudFromRgbd(rgb, depth, cx_, cy_, fx_, fy_, decimation_);
        for(std::vector<boost::shared_ptr<CloudListener> >::iterator iter = cloud_listeners_.begin() ; iter != cloud_listeners_.end() ; iter++) {
            (*iter)->onCloudFrame(cloud_);
        }
    }

    void CloudProvider::addListener(boost::shared_ptr<CloudListener> cloud_listener) {
        cloud_listeners_.push_back(cloud_listener);
    }

    namespace FrameToCloud {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromRgbd(
                const cv::Mat &rgb,
                const cv::Mat &depth,
                float cx, float cy,
                float fx, float fy,
                int decimation) {

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            if(decimation <1)
                return cloud;

            cloud->height = depth.rows / decimation;
            cloud->width = depth.cols / decimation;
            cloud->is_dense = false;
            cloud->resize(cloud->height * cloud->width);

            for(int h = 0 ; h < depth.rows && h / decimation < static_cast<int>(cloud->height); h+=decimation) {
                for(int w = 0 ; w < depth.cols && w / decimation < static_cast<int>(cloud->width); w+=decimation) {

                    pcl::PointXYZRGB &pt = cloud->at((h/decimation)*cloud->width + (w/decimation));

                    pt.b = rgb.at<cv::Vec3b>(h,w)[0];
                    pt.g = rgb.at<cv::Vec3b>(h,w)[1];
                    pt.r = rgb.at<cv::Vec3b>(h,w)[2];

                    float depth_point = static_cast<float>(depth.at<unsigned short>(h,w)) * 0.001f;
                    pt.x = (static_cast<float>(w) - cx) * depth_point / fx;
                    pt.y = (static_cast<float>(h) - cy) * depth_point / fy;
                    pt.z = depth_point;
                }
            }

            return cloud;
        }
    }
}

