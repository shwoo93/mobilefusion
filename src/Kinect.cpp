#include "Kinect.h"


mobilefusion::Kinect::Kinect()
{
    freenect2_ = 0;
    dev_ = 0;
    pipeline_ = 0;
    registration_ = 0;
    listener_ = 0;
}

mobilefusion::Kinect::~Kinect()
{
    dev_->stop();
    dev_->close();

    delete registration_;
}

void mobilefusion::Kinect::init()
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

}

void mobilefusion::Kinect::captureImage(cv::Mat &rgb, cv::Mat &depth)
{
    libfreenect2::FrameMap frames;
    listener_->waitForNewFrame(frames);
    libfreenect2::Frame *rgb_frame = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth_frame = frames[libfreenect2::Frame::Depth];
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    registration_->apply(rgb_frame, depth_frame, &undistorted, &registered);


    cv::Mat rgb_temp((int)registered.height, (int)registered.width, CV_8UC4, registered.data);
    cv::Mat depth_temp((int)depth_frame->height, (int)depth_frame->width , CV_32FC1,depth_frame->data);

    rgb = rgb_temp;
    depth = depth_temp;

    listener_->release(frames);
}


void mobilefusion::Kinect::rgbdFromCloud(
                const  pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                float& cx, float& cy, float& fx, float& fy,
                cv::Mat &depth, cv::Mat &rgb){

        cv::Mat depth_temp(cloud.height,cloud.width,CV_32FC1);
        cv::Mat rgb_temp(cloud.height,cloud.width,CV_8UC3);

        for(unsigned int h = 0; h < cloud.height; h++)
        {
            for(unsigned int w = 0; w < cloud.width; w++)
            {
                rgb_temp.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).b;
                rgb_temp.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
                rgb_temp.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).r;

                float depth = cloud.at(h*cloud.width + w).z;
                depth_temp.at<float>(h,w) = depth;
            }
        }

        depth = depth_temp;
        rgb = rgb_temp;

       /* for(int i=0;i<cloud.size();i++){
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
        }*/

}
pcl::PointCloud<pcl::PointXYZ>::Ptr mobilefusion::Kinect::cloudFromDepth(
                const cv::Mat &depth,
                float cx, float cy,
                float fx, float fy,
                int decimation)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(decimation < 1)
        return cloud;

    cloud->height = depth.rows/decimation;
    cloud->width = depth.cols/decimation;
    cloud->is_dense = false;

    cloud->resize(cloud->height * cloud->width);

    //int count = 0;

    for(int h = 0; h < depth.rows; h+=decimation)
    {
        for(int w=0; w < depth.cols; h+=decimation)
        {
            pcl::PointXYZ & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));

            pcl::PointXYZ ptXYZ = projectDepthTo3D(depth,w,h,cx,cy,fx,fy);
            pt.x = ptXYZ.x;
            pt.y = ptXYZ.y;
            pt.z = ptXYZ.z;
            // ++count;
        }
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr mobilefusion::Kinect::cloudFromRgbd(
                const cv::Mat & rgb, const cv::Mat &depth,
                float cx, float cy, float fx, float fy,
                int decimation)
{

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                cloud->height = depth.rows/decimation;
                cloud->width = depth.cols/decimation;
                cloud->is_dense = false;
                cloud->resize(cloud->height * cloud->width);

               // int width = depth.cols;
               // int height = rgb.rows;
               // pcl::PointCloud<pcl::PointXYZRGB> cloud;

                for(int h=0; h<depth.rows && h/decimation < (int)cloud->height ; h+=decimation)
                {
                    for(int w=0;w<depth.cols && w/decimation < (int)cloud->width ; w+=decimation)
                    {
                        pcl::PointXYZRGB &pt = cloud->at((h/decimation)*(int)cloud->width + (w/decimation));

                        pt.b = rgb.at<cv::Vec3b>(h,w)[0];
                        pt.g = rgb.at<cv::Vec3b>(h,w)[1];
                        pt.r = rgb.at<cv::Vec3b>(h,w)[2];

                        pcl::PointXYZ ptXYZ = projectDepthTo3D(depth,w,h,cx,cy,fx,fy);
                        pt.x = ptXYZ.x;
                        pt.y = ptXYZ.y;
                        pt.z = ptXYZ.z;
                        /*
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
                            cloud->push_back(pcl::PointXYZRGB(p));
                        }
                        */
                    }
                }
                return cloud;
}


pcl::PointXYZ mobilefusion::Kinect::projectDepthTo3D(
                        const cv::Mat & depth,
                        float x, float y,
                        float cx, float cy,
                        float fx, float fy)
{
                pcl::PointXYZ pt;

               // float z_3d = static_cast<float>(z);
                int u = int(x+0.5f);
                int v = int(y+0.5f);
                float depth_tmp  = static_cast<float>(depth.at<unsigned short>(v,u)) *0.001f;
                pt.x = (static_cast<float>(x) - cx) * depth_tmp / fx;
                pt.y = (static_cast<float>(y) - cy) * depth_tmp / fy;
                pt.z = depth_tmp;
                return pt;

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr mobilefusion::Kinect::removeNaNFromPointCloud(
                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*output,indices);
    return output;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr mobilefusion::Kinect::voxelize(
                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
                float voxelSize)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    filter.setInputCloud(cloud);
    filter.filter(*output);
    return output;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr mobilefusion::Kinect::transformPointCloud(
                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
                const mobilefusion::Transform & transform)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud, *output, transform.toEigen4f());
    return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr mobilefusion::Kinect::getICPReadyCloud(
                const cv::Mat &rgb,
                const cv::Mat &depth,
                float fx,
                float fy,
                float cx,
                float cy,
                int decimation,
                float voxel,
                const mobilefusion::Transform & transform)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud = cloudFromRgbd(rgb,depth,cx,cy,fx,fy,decimation);
    if(cloud->size())
    {
        if(voxel>0)
        {
            cloud = voxelize(cloud,voxel);
        }
        if(cloud->size())
        {
            if(!transform.isNull() && !transform.isIdentitiy())
            {
                cloud = transformPointCloud(cloud,transform);
            }
        }
    }
    return cloud;
}


mobilefusion::Transform mobilefusion::Kinect::GetTransMat(
                        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud_source,
                        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud_target,
                        pcl::PointCloud<pcl::PointXYZRGB> & cloud_source_registered)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sourceDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_targetDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);


    //make dense
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*modelCloud, *modelCloud, indices);
    // pcl::removeNaNFromPointCloud(*dataCloud, *dataCloud, indices);

    cloud_source = mobilefusion::Kinect::removeNaNFromPointCloud(cloud_source);
    cloud_target = mobilefusion::Kinect::removeNaNFromPointCloud(cloud_target);

    // pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    //  vg.setInputCloud(modelCloud);
    //  vg.setLeafSize(0.01f, 0.01f, 0.01f);
    //  vg.filter(*modelCloudDownsampled);

    // vg.setInputCloud(dataCloud);
    // vg.setLeafSize(0.01f, 0.01f, 0.01f);
    // vg.filter(*dataCloudDownsampled);

    cloud_sourceDownsampled = mobilefusion::Kinect::voxelize(cloud_source,0.01f);
    cloud_targetDownsampled = mobilefusion::Kinect::voxelize(cloud_target,0.01f);

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
    gicp.setInputSource(cloud_sourceDownsampled);
    gicp.setInputTarget(cloud_targetDownsampled);

    gicp.align(cloud_source_registered);
    return mobilefusion::Transform::fromEigen4f(gicp.getFinalTransformation());
}

mobilefusion::Transform::Transform() : data_(cv::Mat::zeros(3,4,CV_32FC1))
{}

mobilefusion::Transform::Transform(
                float r11, float r12, float r13, float o14,
                float r21, float r22, float r23, float o24,
                float r31, float r32, float r33, float o34)
{
    data_ = (cv::Mat_<float>(3,4) <<
                    r11, r12, r13, o14,
                    r21, r22, r23, o24,
                    r31, r32, r33, o34);
}

bool mobilefusion::Transform::isNull() const
{
    return  ( data_[0] == 0.0f &&
                         data_[1] == 0.0f &&
                         data_[2] == 0.0f &&
                         data_[3] == 0.0f &&
                         data_[4] == 0.0f &&
                         data_[5] == 0.0f &&
                         data_[6] == 0.0f &&
                         data_[7] == 0.0f &&
                         data_[8] == 0.0f &&
                         data_[9] == 0.0f &&
                         data_[10] == 0.0f &&
                         data_[11] == 0.0f);

}

bool mobilefusion::Transform::isIdentity() const
{
         return (data()[0] == 1.0f &&
                         data()[1] == 0.0f &&
                         data()[2] == 0.0f &&
                         data()[3] == 0.0f &&
                         data()[4] == 0.0f &&
                         data()[5] == 1.0f &&
                         data()[6] == 0.0f &&
                         data()[7] == 0.0f &&
                         data()[8] == 0.0f &&
                         data()[9] == 0.0f &&
                         data()[10] == 1.0f &&
                         data()[11] == 0.0f);
}
