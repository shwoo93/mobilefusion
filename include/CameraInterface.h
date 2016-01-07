#ifndef CAMERAINTERFACE_H_
#define CAMERAINTERFACE_H_

#include <opencv2/opencv.hpp>

namespace mobilefusion{
    class CameraInterface
    {
        public:
            CameraInterface();
            ~CameraInterface();
            virtual void captureImage(cv::Mat &rgb, cv::Mat &depth) = 0;
    };
}

#endif
