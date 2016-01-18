#include "KinectRenderer.h"

#include <iostream>

namespace MobileFusion {
    KinectRenderer::KinectRenderer():converter_(new KinectFrameToCloud) {
        cv::namedWindow("rgb");
        cv::namedWindow("depth");
        octree_.setMinWeight(2);
        octree_.setColorByRGB(true);
    }

    KinectRenderer::~KinectRenderer() {
    }

    void KinectRenderer::OnFrame(cv::Mat &rgb, cv::Mat &depth) {
        cv::imshow("rgb", rgb);
        cv::imshow("depth", depth);
        cv::waitKey(10);

        octree_.setInputTSDF(converter_->getTSDF());
        octree_.reconstruct(mesh_);
    }
}
