#include "Util3d.h"

namespace MobileFusion {
    namespace util3d {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromRgbd(
                const cv::Mat &rgb,
                const cv::Mat &depth,
                float cx, float cy,
                float fx, float fy,
                int decimation) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            if (decimation < 1)
                return cloud;

            cloud->height = depth.rows / decimation;
            cloud->width = depth.cols / decimation;
            cloud->is_dense = false;
            cloud->resize(cloud->height * cloud->width);
            for (int h = 0; h < depth.rows && h / decimation < static_cast<int>(cloud->height); h += decimation) {
                for (int w = 0; w < depth.cols && w / decimation < static_cast<int>(cloud->width); w += decimation) {

                    pcl::PointXYZRGB &pt = cloud->at((h/decimation)*cloud->width + (w/decimation));

                    pt.b = rgb.at<cv::Vec3b>(h,w)[0];
                    pt.g = rgb.at<cv::Vec3b>(h,w)[1];
                    pt.r = rgb.at<cv::Vec3b>(h,w)[2];

                    float depth_point = static_cast<float>(depth.at<unsigned short>(h, w)) * 0.001f;
                    pt.x = (static_cast<float>(w) - cx) * depth_point / fx;
                    pt.y = (static_cast<float>(h) - cy) * depth_point / fy;
                    pt.z = depth_point;
                }
            }

            return cloud;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfromDepth(
                const cv::Mat & depth,
                float cx, float cy,
                float fx, float fy,
                int decimation) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

            if(decimation <1)
                return cloud;

            cloud->height = depth.rows/decimation;
            cloud->width = depth.cols/decimation;
            cloud->is_dense = false;

            cloud->resize(cloud->height*cloud->width);

            for(int h = 0 ; h < depth.rows ; h+=decimation)
            {
                for(int w = 0 ; w < depth.cols ; w+=decimation)
                {
                    pcl::PointXYZ & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));

                    pcl::PointXYZ ptXYZ = projectDepthTo3D(depth, w, h, cx, cy, fx, fy);
                    pt.x = ptXYZ.x;
                    pt.y = ptXYZ.y;
                    pt.z = ptXYZ.z;
                }
            }
            return cloud;
        }

        pcl::PointXYZ projectDepthTo3D(
                const cv::Mat &depth,
                float x, float y,
                float cx, float cy,
                float fx, float fy) {
            pcl::PointXYZ pt;

            float depth_point = static_cast<float>(depth.at<unsigned short>(y, x)) * 0.001f;
            pt.x = (static_cast<float>(x) - cx) * depth_point / fx;
            pt.y = (static_cast<float>(y) - cy) * depth_point / fy;
            pt.z = depth_point;

            return pt;
        }


        void rgbdFromCloud(
                const  pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                float &cx, float &cy, float &fx, float &fy,
                cv::Mat &depth, cv::Mat &rgb) {

            cv::Mat depth_temp(cloud.height,cloud.width,CV_32FC1);
            cv::Mat rgb_temp(cloud.height,cloud.width,CV_8UC3);

            for(unsigned int h = 0; h < cloud.height; h++) {
                for(unsigned int w = 0; w < cloud.width; w++) {
                    rgb_temp.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).b;
                    rgb_temp.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
                    rgb_temp.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).r;

                    float depth = cloud.at(h*cloud.width + w).z;
                    depth_temp.at<float>(h,w) = depth;
                }
            }

            depth = depth_temp;
            rgb = rgb_temp;
        }
    }
}
