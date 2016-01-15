#include "KinectRecorder.h"

#include <iostream>

namespace mobilefusion{
	KinectRecorder::KinectRecorder()
	{

	}

	KinectRecorder::~KinectRecorder()
	{

	}

	void KinectRecorder::OnFrame(cv::Mat &rgb, cv::Mat &depth)
	{
		std::cout << "OnFrame()" << std::endl;
	}
}