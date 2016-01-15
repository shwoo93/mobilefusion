#ifndef CAMERAINTERFACE_H_
#define CAMERAINTERFACE_H_

#include <opencv2/opencv.hpp>

namespace mobilefusion{
    class CameraInterface
    {
        public:
            CameraInterface();
<<<<<<< HEAD
            virtual ~CameraInterface();
=======
            ~CameraInterface();
>>>>>>> d90abbe570863f5858036b9a744f61a42d295191
            virtual void captureImage(cv::Mat &rgb, cv::Mat &depth) = 0;
    };
}

#endif
