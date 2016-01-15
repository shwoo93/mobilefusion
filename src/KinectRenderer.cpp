#include "KinectRenderer.h"

#include <iostream>

namespace MobileFusion {
	KinectRenderer::KinectRenderer() {
	    cv::namedWindow("rgb");
	    cv::namedWindow("depth");
	}

	KinectRenderer::~KinectRenderer() {
	}

	void KinectRenderer::OnFrame(cv::Mat &rgb, cv::Mat &depth) {
    	cv::imshow("rgb", rgb);
    	cv::imshow("depth", depth);
        cv::waitKey(10);
	}
}