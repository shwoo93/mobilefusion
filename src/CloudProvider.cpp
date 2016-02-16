#include "CloudProvider.h"

#include "boost/format.hpp"

#include "CloudListener.h"

namespace MobileFusion{
    CloudProvider::CloudProvider()
    : cx_(256.0f)
    , cy_(212.0f)
    , fx_(540.686f)
    , fy_(540.686f)
    , dense_(false)
    , cloud_listeners_() {
    }

    CloudProvider::~CloudProvider() {
    }

    void CloudProvider::setDense(bool dense) {
    	dense_ = dense;
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
        CV_Assert(!rgb.empty() && !depth.empty());
        CV_Assert(rgb.type() == CV_8UC3 && depth.type() == CV_32FC1);

        if(dense_) {
	        int width = rgb.cols;
	        int height = rgb.rows;

	        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	        cloud->is_dense = true;

	        for(int i = 0 ; i < width; ++i) {
	            for(int j = 0 ; j < height; ++j) {
	                pcl::PointXYZRGB point;

	                //depth in meters
	                if(depth.at<float>(j, i) == 0.0f) {
	                    continue;
	                }
	                else {
	                    point.z = depth.at<float>(j, i) * 0.001f;
	                }
	                point.x = (static_cast<float>(i) - cx_) * point.z / fx_;
	                point.y = (static_cast<float>(j) - cy_) * point.z / fy_;

	                cv::Vec3b v(rgb.at<cv::Vec3b>(j, i));
	                point.b = v[0];
	                point.g = v[1];
	                point.r = v[2];

	                cloud->push_back(point);
	            }
	        }
	        return cloud;
	    }
	    else {
	    	int width = rgb.cols;
	    	int height = rgb.rows;

	        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	        cloud->width = width;
	        cloud->height = height;
	        cloud->resize (width * height);
	        cloud->is_dense = false;

            for(int w=0; w < width; ++w) {
	        	for(int h = 0; h < height; ++h) {

	                pcl::PointXYZRGB &pt = cloud->at(w, h);

	                if(depth.at<float>(h, w) == 0.0f) {
	                    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
	                    pt.r = pt.g = pt.b = std::numeric_limits<float>::quiet_NaN ();
	                    continue;
	                }
	                else {
	                    pt.z = depth.at<float>(h, w) * 0.001f;
	                }

	                pt.x = (static_cast<float>(w) - cx_) * pt.z / fx_;
	                pt.y = (static_cast<float>(h) - cy_) * pt.z / fy_;

	                cv::Vec3b v(rgb.at<cv::Vec3b>(h, w));
	                pt.b = v[0];
	                pt.g = v[1];
	                pt.r = v[2];
	            }
	        }

        	return cloud;
        }
    }
}

