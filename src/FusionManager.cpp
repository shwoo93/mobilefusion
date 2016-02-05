#include "FusionManager.h"

#include "CloudNormalProvider.h"

#include <boost/format.hpp>
#include <boost/chrono.hpp>

#include <pcl/PolygonMesh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/octree.h>
#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <eigen_extensions/eigen_extensions.h>

namespace MobileFusion {
    FusionManager::FusionManager()
    : renderer_("compare")
    , registerer_()
    , tsdf_()
    , cloud_dirty_(false)
    , cloud_()
    , update_count_ (0)
    , octree_ () {
        octree_.reset();
    }

    FusionManager::~FusionManager() {
    }

    void FusionManager::update() {

        //if(!cloud_dirty_)
        //    return;

        //++update_count_;

        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_;
        //cloud_dirty_ = false;

        //pcl::PointCloud<pcl::Normal>::Ptr normal = CloudNormalProvider::computeNormal(cloud,5);
        //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudConcatenated (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        //pcl::concatenateFields (*cloud, *normal, *cloudConcatenated);

        //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr raytraced (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        ////first time
        //if(update_count_ == 1) {
        //    //tsdf_.integrateCloud (*cloud, *normal); //
        //    //raytraced = tsdf_.renderColoredView (); //
        //    octree_.integrateCloud (*cloud, *normal);
        //    raytraced = octree_.renderColoredView();
        //    registerer_.setSourceCloud (raytraced);
        //    std::cout<<raytraced->size()<<std::endl;
        //}

        //else {
        //    std::cout<<"a"<<std::endl;
        //    registerer_.setTargetCloud (cloudConcatenated);
        //    std::cout<<"cloudconcatenate:"<<cloudConcatenated->size()<<std::endl;
        //    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_downsampled (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        //    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_registered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        //    registerer_.getIcpResultCloud (5.f, cloud_target_downsampled, cloud_source_registered);
        //    renderer_.onCloudFrame(cloud_target_downsampled, cloud_source_registered);
        //    //tsdf_.integrateCloud (*cloud, *normal, registerer_.getCameraPose ().inverse ()); //
        //    octree_.integrateCloud (*cloud, *normal, registerer_.getCameraPose ().inverse());
        //    std::cout<<registerer_.getCameraPose().matrix()<<std::endl;
        //    //raytraced = tsdf_.renderColoredView (registerer_.getCameraPose().inverse ()); //
        //    raytraced = octree_.renderColoredView (registerer_.getCameraPose().inverse ());
        //    std::cout<<"raytraced"<<raytraced->size()<<std::endl;
        //    registerer_.setSourceCloud (raytraced);


        //static int index = 1;
        //if(!cloud_dirty_)
        //    return;

        //++update_count_;

        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_));
        //cloud_dirty_ = false;

        //pcl::PointCloud<pcl::Normal>::Ptr normal = CloudNormalProvider::computeNormal(cloud, 0.05f);
        //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_concatenated (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        //pcl::concatenateFields (*cloud, *normal, *cloud_concatenated);

        //if(update_count_ == 1) {
        //    std::string count = str(boost::format("%1%") % index);
        //    tsdf_.integrateCloud (*cloud,*normal);
        //    //octree_.integrateCloud (*cloud, *normal);
        //    registerer_.setTargetCloud (cloud_concatenated);
        //    index++;
        //}

        //else {
        //    std::string count = str(boost::format("%1%") % index);
        //    std::cout<< count <<std::endl;
        //    registerer_.setSourceCloud (cloud_concatenated);
        //    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_downsampled (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        //    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_registered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        //    registerer_.getIcpResultCloud (0.5f, cloud_target_downsampled, cloud_source_registered);
        //    //octree_.integrateCloud (*cloud, *normal, registerer_.getCameraPose());
        //    tsdf_.integrateCloud (*cloud, *normal, registerer_.getCameraPose());
        //    registerer_.setTargetCloud (cloud_concatenated);
        //    index++;
        //}

        //tsdf_.constructMesh ();

        //if(!cloud_dirty_) {
        //    return;
        //}
        //else {
        //    cloud_dirty_ = false;
        //}

        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        //cloud = cloud_;

        //static pcl::PointCloud<pcl::PointXYZRGB>::Ptr target;
        //static Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();

        //pcl::PointCloud<pcl::Normal>::Ptr normal = CloudNormalProvider::computeNormal(cloud);
        ////pcl::PointCloud<pcl::Normal>::Ptr empty_normal (new pcl::PointCloud<pcl::Normal>);

        //if(update_count_ == 0) {
        //    target = cloud;
        //    tsdf_.integrateCloud(*target,*normal);
        //    //octree_.integrateCloud(*target,*normal);
        //}

        //else {
        //    Eigen::Matrix4f pair_transform;
        //    registerer_.pairAlign(cloud, target, pair_transform);
        //    global_transform *= pair_transform;
        //    //matrix4f to affine3d
        //    Eigen::Affine3d global_transform_affine (global_transform.cast<double> ());
        //    tsdf_.integrateCloud(*cloud, *normal, global_transform_affine);
        //    //octree_.integrateCloud(*cloud, *normal, global_transform_affine);
        //    target = cloud;
        //}

        //++update_count_;
        //tsdf_.constructMesh ();
        boost::chrono::system_clock::time_point start, end;

        if(!cloud_dirty_) {
            return;
        }
        else {
            cloud_dirty_ = false;
        }

        std::cout << "FusionManager::update()" << std::endl;

        //using cpu_tsdf initialization part
        static cpu_tsdf::TSDFVolumeOctree::Ptr tsdf(new cpu_tsdf::TSDFVolumeOctree);
        static cpu_tsdf::MarchingCubesTSDFOctree mc;

        if(update_count_ == 0) {

            tsdf->setGridSize(3., 3., 3.);
            tsdf->setResolution(256, 256, 256);
            tsdf->setIntegrateColor(true);
            tsdf->reset();
            tsdf->setImageSize(512, 424);
            tsdf->setCameraIntrinsics(540.686f, 540.686f, 256.0f, 212.0f);
            mc.setMinWeight(2);
            mc.setColorByRGB(true);
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_));
        pcl::PointCloud<pcl::Normal>::Ptr normal = CloudNormalProvider::computeNormal(cloud, 4);
        pcl::PointCloud<pcl::Normal>::Ptr empty_normal (new pcl::PointCloud<pcl::Normal>());

        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGB>);
        static Eigen::Matrix4f global_transformation = Eigen::Matrix4f::Identity();

        if(update_count_ == 0) {
            target = cloud;
            tsdf->integrateCloud (*target, *normal);
            //tsdf->integrateCloud (*target, *empty_normal);
        }

        else {

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_source (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_target (new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
            uniform_sampling.setRadiusSearch (0.05f);

            uniform_sampling.setInputCloud(cloud);
            uniform_sampling.filter(*keypoints_source);

            uniform_sampling.setInputCloud(target);
            uniform_sampling.filter(*keypoints_target);

            std::cout << "Number of cloud source : " << cloud->size() << std::endl;
            std::cout << "Number of cloud target : " << target->size() << std::endl;

            std::cout << "Number of sampled source : " << keypoints_source->size() << std::endl;
            std::cout << "Number of sampled target : " << keypoints_target->size() << std::endl;

            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
            //gicp.setTransformationEpsilon (1e-6);
            //gicp.setMaxCorrespondenceDistance (0.1);
            //gicp.setMaximumIterations (30);

            gicp.setInputSource(keypoints_source);
            gicp.setInputTarget(keypoints_target);

            pcl::PointCloud<pcl::PointXYZRGB> aligned;
            start = boost::chrono::system_clock::now();
            gicp.align(aligned);
            end = boost::chrono::system_clock::now();
            std::cout << "time for gicp : " << boost::chrono::duration_cast<boost::chrono::milliseconds>(end - start) << std::endl;


            if(gicp.hasConverged()) {
                std::cout << "score: " << gicp.getFitnessScore() << std::endl;

                target = cloud;

                global_transformation *= gicp.getFinalTransformation();
                Eigen::Affine3d trans (global_transformation.cast<double>());
                start = boost::chrono::system_clock::now();
                tsdf->integrateCloud (*cloud, *normal, trans);
                end = boost::chrono::system_clock::now();
                std::cout << "time for integrate cloud : " << boost::chrono::duration_cast<boost::chrono::milliseconds>(end - start) << std::endl;
                //tsdf->integrateCloud (*cloud, *empty_normal, trans);
            }

            else
                std::cout<< "GICP did not converged!" << std::endl;
        }

        ++update_count_;

        //marching cube, extract surface from tsdf volume
        mc.setInputTSDF (tsdf);
        pcl::PolygonMesh mesh;
        start = boost::chrono::system_clock::now();
        mc.reconstruct (mesh);
        end = boost::chrono::system_clock::now();
        std::cout << "time for mesh reconstruct : " << boost::chrono::duration_cast<boost::chrono::milliseconds>(end - start) << std::endl;

        std::string mesh_name = str(boost::format("/home/vllab/Desktop/mesh/test%1%") % update_count_);
        pcl::io::savePolygonFilePLY(mesh_name, mesh);
    }

    void FusionManager::onFrame(const cv::Mat& rgb, const cv::Mat& depth) {
    }

    void FusionManager::onCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        cloud_ = cloud;
        cloud_dirty_ = true;
    }
}

