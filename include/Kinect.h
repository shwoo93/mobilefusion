#ifndef KINECT_H_
#define KINECT_H_

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/cloud_viewer.h>

#include "CameraInterface.h"


namespace mobilefusion {
    class Kinect : public CameraInterface {
        public:
            Kinect();
            ~Kinect();
            void init();
            void captureImage(cv::Mat &rgb, cv::Mat &depth);
            //void rgbdFromCloud(
            //    pcl::PointCloud<pcl::PointXYZRGB> cloud,
            //    float cx, float cy,
            //    float fx, float fy,
            //    cv::Mat &depth, cv::Mat &rgb);
            //pcl::PointCloud<pcl::PointXYZRGB> cloudFromRgbd(
            //        cv::Mat depth, cv::Mat rgb,
            //        float cx, float cy,
            //        float fx, float fy);
            //pcl::PointXYZ projectPoint(int x, int y, int z);
            //Eigen::Matrix4f GetTransMat(
            //        pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloud,
            //        pcl::PointCloud<pcl::PointXYZRGB>::Ptr dataCloud);

        private:
            libfreenect2::Freenect2 *freenect2_;
            libfreenect2::Freenect2Device *dev_;
            libfreenect2::PacketPipeline *pipeline_;
            libfreenect2::Registration *registration_;
            libfreenect2::SyncMultiFrameListener *listener_;
            //pcl::PointCloud<pcl::PointXYZRGB> cloud_;
            //float cx_, cy_, fx_, fy_;
            //cv::Mat depth_, rgb_;
            //int x_ , y_, z_;
            //float max_depth_;
    };
}

#endif
