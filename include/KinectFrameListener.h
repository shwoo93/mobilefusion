#ifndef __KINECT_FRAME_LISTENER__
#define __KINECT_FRAME_LISTENER__

#include <opencv2/opencv.hpp>

namespace MobileFusion {
	class KinectFrameListener {
	public:
		KinectFrameListener() {};
		virtual ~KinectFrameListener() {};
		virtual void OnFrame(cv::Mat &rgb, cv::Mat &depth) = 0;
	};
}



#endif