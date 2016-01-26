#include "KinectRenderer.h"

#include <iostream>

namespace MobileFusion {
    KinectRenderer::KinectRenderer() {
        cv::namedWindow("rgb");
        cv::namedWindow("depth");
    }

    KinectRenderer::~KinectRenderer() {
        cv::destroyWindow("rgb");
        cv::destroyWindow("depth");
    }

    void KinectRenderer::onFrame(const cv::Mat &rgb, const cv::Mat &depth) {
        cv::imshow("rgb", rgb);
        cv::imshow("depth", depth);
        cv::waitKey(1);
    }
}
