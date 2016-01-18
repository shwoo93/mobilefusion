#ifndef __KINECT_RENDERER_H__
#define __KINECT_RENDERER_H__

//#include "KinectFrameListener.h"
#include "KinectFrameToCloud.h"

namespace MobileFusion {
    class KinectRenderer : public KinectFrameListener {
        public:
            KinectRenderer();
            ~KinectRenderer();
            void OnFrame(cv::Mat &rgb, cv::Mat &depth);
        private:
            cpu_tsdf::MarchingCubesTSDFOctree octree_;
            pcl::PolygonMesh mesh_;
            boost::shared_ptr<KinectFrameToCloud> converter_;
    };
}

#endif
