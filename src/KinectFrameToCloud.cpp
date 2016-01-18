#include "KinectFrameToCloud.h"

namespace MobileFusion{
    KinectFrameToCloud::KinectFrameToCloud()
        : cx_(256.0f)
          , cy_(212.0f)
          , fx_(540.686f)
          , fy_(540.686f)
          , decimation_(5)
          , cloud_num_(0)
          , mat(Eigen::Matrix4f::Identity())
          , cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
          , registration_(new KinectRegistration())
          , tsdf(new cpu_tsdf::TSDFVolumeOctree) {
              tsdf->setGridSize(1.,1.,1.);
              tsdf->setResolution(128, 128, 128);
              tsdf->setIntegrateColor(false);
              tsdf->reset();
              transformations_.push_back(mat);
          }

    KinectFrameToCloud::~KinectFrameToCloud() {
    }

    void KinectFrameToCloud::setCameraIntrinsic(float cx, float cy, float fx, float fy) {
        cx_ = cx;
        cy_ = cy;
        fx_ = fx;
        fy_ = fy;
    }

    void KinectFrameToCloud::setDecimation(int decimation) {
        decimation_ = decimation;
    }

    int KinectFrameToCloud::getCloudNum() {
        return cloud_num_;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> KinectFrameToCloud::getClouds() {
        return clouds_;
    }
    std::vector<Eigen::Matrix4f> KinectFrameToCloud::getTransMat() {
        return transformations_;
    }
    cpu_tsdf::TSDFVolumeOctree::Ptr KinectFrameToCloud::getTSDF() {
        return tsdf;
    }

    void KinectFrameToCloud::addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
        clouds_.push_back(cloud);
        ++cloud_num_;
    }

    void KinectFrameToCloud::OnFrame(cv::Mat &rgb, cv::Mat &depth) {
        static int i=0;
        cloud_ = MobileFusion::FrameToCloud::cloudFromRgbd(rgb, depth, cx_, cy_, fx_, fy_, decimation_);
        addPointCloud(cloud_);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if(clouds_.size()>=2){
            //registration_->setVoxelSize(0.1f);
            Eigen::Matrix4d mat_4d(mat.cast<double>());
            Eigen::Affine3d affine(mat_4d);
            tsdf->integrateCloud(*clouds_[i],*empty_cloud,affine);
            mat*=registration_->getIcpTransformation(clouds_[i+1],clouds_[i]);
            transformations_.push_back(mat);
        }
    }

    //void KinectFrameToCloud::OnCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    //    clouds_.push_back(cloud);
    //    ++cloud_num_;
    //}

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

