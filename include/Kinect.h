#ifndef KINECT_H_
#define KINECT_H_

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/highgui/highgui.hpp>

#include <cpu_tsdf/octree.h>
#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <eigen_extensions/eigen_extensions.h>

#include <vector>
#include "CameraInterface.h"

class Transform;

namespace mobilefusion {

   // class Transform {
   //     public:
   //         Transform();
   //         Transform(float r11, float r12, float r13, float o14,
   //                   float r21, float r22, float r23, float o24,
   //                   float r31, float r32, float r33, float o34);
   //         bool isNull() const;
   //         bool isIdentity() const;
   //         Eigen::Matrix4f toEigen4f() const;
   //         Transform fromEigen4f(const Eigen::Matrix4f & matrix);
   //     private:
   //         cv::Mat data_;
   // };

    class Kinect : public CameraInterface {
        public:
            Kinect();
            ~Kinect();
            void init();
            void captureImage(cv::Mat &rgb, cv::Mat &depth);
            void rgbdFromCloud(
                const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                float& cx, float& cy,
                float& fx, float& fy,
                cv::Mat &depth, cv::Mat &rgb);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDepth(
                            const cv::Mat &depth,
                            float cx, float cy,
                            float fx, float fy,
                            int decimation);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromRgbd(
                    const  cv::Mat& rgb, const cv::Mat& depth,
                    float cx, float cy,
                    float fx, float fy,
                    int decimation);
            pcl::PointXYZ projectDepthTo3D(
                            const cv::Mat &depth,
                            float x, float y,
                            float cx, float cy,
                            float fx, float fy);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNaNFromPointCloud(
                            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
                            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                            float voxelSize);
           // pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(
           //                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
           //                 const Transform & transform);
           // pcl::PointCloud<pcl::PointXYZRGB>::Ptr getICPReadyCloud(
           //                 const cv::Mat &rgb,
           //                 const cv::Mat &depth,
           //                 const float fx,
           //                 const float fy,
           //                 const float cx,
           //                 const float cy,
           //                 int decimation,
           //                 float voxel,
           //                 const Transform & transform);
            Eigen::Matrix4f GetTransMat(
                    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud_source,
                    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud_target,
                    pcl::PointCloud<pcl::PointXYZRGB> & cloud_source_registered);

        private:
            libfreenect2::Freenect2 *freenect2_;
            libfreenect2::Freenect2Device *dev_;
            libfreenect2::PacketPipeline *pipeline_;
            libfreenect2::Registration *registration_;
            libfreenect2::SyncMultiFrameListener *listener_;
            //pcl::PointCloud<pcl::PointXYZRGB> cloud_;
            //float cx_, cy_, fx_, fy_;
            // cv::Mat depth, rgb;
            //int x_ , y_, z_;
            //float max_depth_;
    };



}

#endif
