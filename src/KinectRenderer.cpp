#include "KinectRenderer.h"

#include <iostream>

namespace mobilefusion{
	KinectRenderer::KinectRenderer()
	{
	    cv::namedWindow("rgb");
	    cv::namedWindow("depth");
	}

	KinectRenderer::~KinectRenderer()
	{

	}

	void KinectRenderer::OnFrame(cv::Mat &rgb, cv::Mat &depth)
	{
		std::cout << "Render" << std::endl;
		if(depth.empty())
			return;
        cv::imshow("rgb", rgb);
        cv::imshow("depth", depth);
        cv::waitKey(1);
	}
}