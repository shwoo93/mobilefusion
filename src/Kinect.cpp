#include "Kinect.h"

using namespace mobilefusion;

Kinect::Kinect()
{
    freenect2_ = 0;
    dev_ = 0;
    pipeline_ = 0;
    registration_ = 0;
    listener_ = 0;
}

Kinect::~Kinect()
{
    dev_->stop();
    dev_->close();

    delete registration_;
}

void Kinect::init()
{
    freenect2_ = new libfreenect2::Freenect2();
    if (freenect2_->enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        exit(1);
    }

    std::string serial = freenect2_->getDefaultDeviceSerialNumber();
    pipeline_ = new libfreenect2::OpenCLPacketPipeline();
    dev_ = freenect2_->openDevice(serial, pipeline_);


    if (dev_ == 0)
    {
        std::cout << "failure opening device!" << std::endl;
    }

    listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

    dev_->setColorFrameListener(listener_);
    dev_->setIrAndDepthFrameListener(listener_);

    dev_->start();

    std::cout << "device serial: " << dev_->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev_->getFirmwareVersion() << std::endl;

    registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

    std::cout << "end init" << std::endl;
}

void  Kinect::captureImage(cv::Mat &rgb, cv::Mat &depth)
{
    libfreenect2::FrameMap frames;
    listener_->waitForNewFrame(frames);
    libfreenect2::Frame *rgb_frame = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth_frame = frames[libfreenect2::Frame::Depth];
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    registration_->apply(rgb_frame, depth_frame, &undistorted, &registered);

    cv::Mat rgb_temp((int)rgb_frame->height, (int)rgb_frame->width, CV_8UC4, rgb_frame->data);
    cv::Mat depth_temp((int)depth_frame->height, (int)depth_frame->width, CV_32FC1, depth_frame->data);

    //rgb = rgb_temp;
    depth = depth_temp;

    listener_->release(frames);
}

/*
void Kinect::rgbdFromCloud(
                pcl::PointCloud<pcl::PointXYZRGB> cloud,
                float cx, float cy, float fx, float fy,
                cv::Mat &depth, cv::Mat &rgb){

        cv::Mat depth_temp = cv::Mat::zeros(depth.row,depth.col,CV_32FC1);
        cv::Mat rgb_temp = cv::Mat::zeros(rgb.row,rgb.col,CV_8UC3);

        for(int i=0;i<cloud.size();i++){
            float ptx = cloud.points[i].x;
            float pty = cloud.points[i].y;
            float ptz = cloud.points[i].z;
            uint8_t ptr = cloud.points[i].r;
            uint8_t ptg = cloud.points[i].g;
            uint8_t ptb = cloud.points[i].b;

            if(isnan(ptx))
                continue;

            int x = static_cast<int>(ptx*fx/ptz +cx +0.5f);
            int y = static_cast<int>(pty*fy/ptz +cy +0.5f);
            float z = ptz*1000.0f;

            if(x>=0 && y>=0 && x<=640 && y<=480){
                depth_tmp.at<float>(y,x) = z;
                rgb_temp.at<cv::Vec3b>(y,x)[0] = ptb;
                rgb_temp.at<cv::Vec3b>(y,x)[1] = ptg;
                rgb_temp.at<cv::Vec3b>(y,x)[2] = ptr;
            }
        }
        depth = depth_temp;
        rgb = rgb_temp;
}

pcl::PointCloud<pcl::PointXYZRGB> Kinect::cloudFromRgbd(
                cv::Mat depth, cv::Mat rgb,
                float cx, float cy, float fx, float fy){

                int width = rgb.cols;
                int height = rgb.rows;
                pcl::PointCloud<pcl::PointXYZRGB> cloud;

                for(int h=0;h<height;h++){
                    for(int w=0;w<width;w++){
                        float z = static_cast<float>(depth.at<uint16_t>(h,w))/1000.0f;
                        if(z>0.0f){
                            float x = (static_cast<float>(w) - cx)* z /fx;
                            float y = (static_cast<floay>(h) - cy)* z /fy;
                            pcl::PointXYZRGB p;
                            p.x = x; p.y=y; p.z=z;
                            uint8_t b = rgb.at<cv::Vec3b>(h,w)[0];
                            uint8_t g = rgb.at<cv::Vec3b>(h,w)[1];
                            uint8_t r = rgb.at<cv::Vec3b>(h,w)[2];
                            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                            p.rgb = *reinterpret_cast<float*>(*rgb);
                            cloud.push_back(pcl::PointXYZRGB(p));
                        }
                    }
                }
                return cloud;
}

pcl::PointXYZ Kinect::projectPoint(int x, int y, int z){
                float z_3d = static_cast<float>(z);
                float x_3d = (static_cast<float>(x) - 256.0f) * z_3d / 540.686f;
                float y_3d = (static_cast<floay>(y) - 212.0f) * z_3d / 540.686f;
                return pcl::PointXYZ (x_3d, y_3d, z_3d);
}

Eigen::Matrix4f Kinect::GetTransMat(
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloud,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr dataCloud){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloudDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dataCloudDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

    //make dense
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*modelCloud, *modelCloud, indices);
    pcl::removeNaNFromPointCloud(*dataCloud, *dataCloud, indices);

    pcl::VoexlGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(modelCloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*modelCloudDownsampled);

    vg.setInputCloud(dataCloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*dataCloudDownsampled);

    pcl::IterativeClosetPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
    gicp.setInputCloud(dataCloudDownsampled);
    gicp.setInputTarget(modelCloudDownsampled);

    gicp.align(*transformed);

    pcl::transformPointCloud(*dataCloudDownsampled, *transformed, gicp.getFinalTransformation());

    return gicp.getFinalTransformation();


    //pcl::visualization::CloudViewer viewer("Viewer");
    //viewer.addPointCloud(modelCloudDownsampled,"model");
    //viewer.addPointCloud(transformedm,"transformed");

    //viewer.runOnVisualizationThreadOnce(viewerOnceOff);
    //while(viewer.wasStopped())
    //{}
}
*/
