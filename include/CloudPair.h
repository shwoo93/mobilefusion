#ifndef __ICP_READY_CLOUD_PAIR_H__
#define __ICP_READY_CLOUD_PAIR_H__

#include <vector>

namespace MobileFusion {
    class ICPReadyCloudPair {
        public:
            ICPReadyCloudPair();
            ~ICPReadyCloudPair();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr getcloud1();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr getcloud2();
            void setcloud1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetDownsampled);
            void setcloud2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceDownsampled);
            bool isFull();
        private:
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_;//targetDownsampled
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_;//sourceDownsampled
            std::vector<pcl::PointXYZRGB> clouds_;
    };
}

#endif
