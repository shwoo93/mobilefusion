#ifndef __KINECT_FRAME_TO_CLOUD_H__
#define __KINECT_FRAME_TO_CLOUD_H__

#include "KinectFrameListener.h"

namespace MobileFusion {
    class KinectFrameToCloud :public KinectFrameListener {
        public:
            KinectFrameToCloud();
            ~KinectFrameToCloud();
            void setCameraIntrinsic(float cx, float cy, float fx, float fy);
            void setDecimation(int decimation);
            int getCloudNum();
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getClouds();
            void addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
            void OnFrame(cv::Mat &rgb, cv::Mat &depth);
            void OnCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
        private:
            float cx_;
            float cy_;
            float fx_;
            float fy_;
            int decimation_;
            int cloud_num_;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_;

    };

    namespace FrameToCloud {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromRgbd(
                const cv::Mat &rgb,
                const cv::Mat &depth,
                float cx, float cy,
                float fx, float fy,
                int decimation);
    }
}

#endif
